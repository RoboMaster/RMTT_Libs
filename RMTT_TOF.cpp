// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

// This file is copied from github
// https://github.com/pololu/vl53l0x-arduino.

#include <Wire.h>
#include "RMTT_Libs.h"

// Defines /////////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0b0101001

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)(millis() - timeout_start_ms) > io_timeout))

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Constructors ////////////////////////////////////////////////////////////////

RMTT_TOF::RMTT_TOF()
  : address(ADDRESS_DEFAULT)
  , io_timeout(0) // no timeout
  , did_timeout(false)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void RMTT_TOF::SetAddress(uint8_t new_addr)
{
  WriteReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
  address = new_addr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool RMTT_TOF::Init(bool io_2v8)
{
  // check model ID register (value specified in datasheet)
  if (ReadReg(IDENTIFICATION_MODEL_ID) != 0xEE) { return false; }

  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    WriteReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
    ReadReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  WriteReg(0x88, 0x00);

  WriteReg(0x80, 0x01);
  WriteReg(0xFF, 0x01);
  WriteReg(0x00, 0x00);
  stop_variable = ReadReg(0x91);
  WriteReg(0x00, 0x01);
  WriteReg(0xFF, 0x00);
  WriteReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  WriteReg(MSRC_CONFIG_CONTROL, ReadReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  SetSignalRateLimit(0.25);

  WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!GetSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  WriteReg(0xFF, 0x01);
  WriteReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  WriteReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  WriteReg(0xFF, 0x00);
  WriteReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  WriteMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  WriteReg(0xFF, 0x01);
  WriteReg(0x00, 0x00);

  WriteReg(0xFF, 0x00);
  WriteReg(0x09, 0x00);
  WriteReg(0x10, 0x00);
  WriteReg(0x11, 0x00);

  WriteReg(0x24, 0x01);
  WriteReg(0x25, 0xFF);
  WriteReg(0x75, 0x00);

  WriteReg(0xFF, 0x01);
  WriteReg(0x4E, 0x2C);
  WriteReg(0x48, 0x00);
  WriteReg(0x30, 0x20);

  WriteReg(0xFF, 0x00);
  WriteReg(0x30, 0x09);
  WriteReg(0x54, 0x00);
  WriteReg(0x31, 0x04);
  WriteReg(0x32, 0x03);
  WriteReg(0x40, 0x83);
  WriteReg(0x46, 0x25);
  WriteReg(0x60, 0x00);
  WriteReg(0x27, 0x00);
  WriteReg(0x50, 0x06);
  WriteReg(0x51, 0x00);
  WriteReg(0x52, 0x96);
  WriteReg(0x56, 0x08);
  WriteReg(0x57, 0x30);
  WriteReg(0x61, 0x00);
  WriteReg(0x62, 0x00);
  WriteReg(0x64, 0x00);
  WriteReg(0x65, 0x00);
  WriteReg(0x66, 0xA0);

  WriteReg(0xFF, 0x01);
  WriteReg(0x22, 0x32);
  WriteReg(0x47, 0x14);
  WriteReg(0x49, 0xFF);
  WriteReg(0x4A, 0x00);

  WriteReg(0xFF, 0x00);
  WriteReg(0x7A, 0x0A);
  WriteReg(0x7B, 0x00);
  WriteReg(0x78, 0x21);

  WriteReg(0xFF, 0x01);
  WriteReg(0x23, 0x34);
  WriteReg(0x42, 0x00);
  WriteReg(0x44, 0xFF);
  WriteReg(0x45, 0x26);
  WriteReg(0x46, 0x05);
  WriteReg(0x40, 0x40);
  WriteReg(0x0E, 0x06);
  WriteReg(0x20, 0x1A);
  WriteReg(0x43, 0x40);

  WriteReg(0xFF, 0x00);
  WriteReg(0x34, 0x03);
  WriteReg(0x35, 0x44);

  WriteReg(0xFF, 0x01);
  WriteReg(0x31, 0x04);
  WriteReg(0x4B, 0x09);
  WriteReg(0x4C, 0x05);
  WriteReg(0x4D, 0x04);

  WriteReg(0xFF, 0x00);
  WriteReg(0x44, 0x00);
  WriteReg(0x45, 0x20);
  WriteReg(0x47, 0x08);
  WriteReg(0x48, 0x28);
  WriteReg(0x67, 0x00);
  WriteReg(0x70, 0x04);
  WriteReg(0x71, 0x01);
  WriteReg(0x72, 0xFE);
  WriteReg(0x76, 0x00);
  WriteReg(0x77, 0x00);

  WriteReg(0xFF, 0x01);
  WriteReg(0x0D, 0x01);

  WriteReg(0xFF, 0x00);
  WriteReg(0x80, 0x01);
  WriteReg(0x01, 0xF8);

  WriteReg(0xFF, 0x01);
  WriteReg(0x8E, 0x01);
  WriteReg(0x00, 0x01);
  WriteReg(0xFF, 0x00);
  WriteReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  WriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  WriteReg(GPIO_HV_MUX_ACTIVE_HIGH, ReadReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = GetMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  SetMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!PerformSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!PerformSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Write an 8-bit register
void RMTT_TOF::WriteReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Write a 16-bit register
void RMTT_TOF::WriteReg16Bit(uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF); // value high byte
  Wire.write( value       & 0xFF); // value low byte
  last_status = Wire.endTransmission();
}

// Write a 32-bit register
void RMTT_TOF::WriteReg32Bit(uint8_t reg, uint32_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write((value >> 24) & 0xFF); // value highest byte
  Wire.write((value >> 16) & 0xFF);
  Wire.write((value >>  8) & 0xFF);
  Wire.write( value        & 0xFF); // value lowest byte
  last_status = Wire.endTransmission();
}

// Read an 8-bit register
uint8_t RMTT_TOF::ReadReg(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  value = Wire.read();

  return value;
}

// Read a 16-bit register
uint16_t RMTT_TOF::ReadReg16Bit(uint8_t reg)
{
  uint16_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2);
  value  = (uint16_t)Wire.read() << 8; // value high byte
  value |=           Wire.read();      // value low byte

  return value;
}

// Read a 32-bit register
uint32_t RMTT_TOF::ReadReg32Bit(uint8_t reg)
{
  uint32_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)4);
  value  = (uint32_t)Wire.read() << 24; // value highest byte
  value |= (uint32_t)Wire.read() << 16;
  value |= (uint16_t)Wire.read() <<  8;
  value |=           Wire.read();       // value lowest byte

  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void RMTT_TOF::WriteMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
  Wire.beginTransmission(address);
  Wire.write(reg);

  while (count-- > 0)
  {
    Wire.write(*(src++));
  }

  last_status = Wire.endTransmission();
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void RMTT_TOF::ReadMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, count);

  while (count-- > 0)
  {
    *(dst++) = Wire.read();
  }
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool RMTT_TOF::SetSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  WriteReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
float RMTT_TOF::GetSignalRateLimit(void)
{
  return (float)ReadReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool RMTT_TOF::SetMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  GetSequenceStepEnables(&enables);
  GetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint32_t final_range_timeout_mclks =
      TimeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    WriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      EncodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t RMTT_TOF::GetMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  GetSequenceStepEnables(&enables);
  GetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool RMTT_TOF::SetVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  GetSequenceStepEnables(&enables);
  GetSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    WriteReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      TimeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    WriteReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      EncodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      TimeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    WriteReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        WriteReg(0xFF, 0x01);
        WriteReg(ALGO_PHASECAL_LIM, 0x30);
        WriteReg(0xFF, 0x00);
        break;

      case 10:
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        WriteReg(0xFF, 0x01);
        WriteReg(ALGO_PHASECAL_LIM, 0x20);
        WriteReg(0xFF, 0x00);
        break;

      case 12:
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        WriteReg(0xFF, 0x01);
        WriteReg(ALGO_PHASECAL_LIM, 0x20);
        WriteReg(0xFF, 0x00);
        break;

      case 14:
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        WriteReg(0xFF, 0x01);
        WriteReg(ALGO_PHASECAL_LIM, 0x20);
        WriteReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    WriteReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      TimeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    WriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      EncodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  SetMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = ReadReg(SYSTEM_SEQUENCE_CONFIG);
  WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  PerformSingleRefCalibration(0x0);
  WriteReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t RMTT_TOF::GetVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(ReadReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(ReadReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void RMTT_TOF::StartContinuous(uint32_t period_ms)
{
  RMTT_I2C_BUSY_LOCK();

  WriteReg(0x80, 0x01);
  WriteReg(0xFF, 0x01);
  WriteReg(0x00, 0x00);
  WriteReg(0x91, stop_variable);
  WriteReg(0x00, 0x01);
  WriteReg(0xFF, 0x00);
  WriteReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = ReadReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    WriteReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    WriteReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    WriteReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
  RMTT_I2C_BUSY_UNLOCK();
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void RMTT_TOF::StopContinuous(void)
{
  RMTT_I2C_BUSY_LOCK();

  WriteReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  WriteReg(0xFF, 0x01);
  WriteReg(0x00, 0x00);
  WriteReg(0x91, 0x00);
  WriteReg(0x00, 0x01);
  WriteReg(0xFF, 0x00);
  RMTT_I2C_BUSY_UNLOCK();
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t RMTT_TOF::ReadRangeContinuousMillimeters(void)
{
  RMTT_I2C_BUSY_LOCK();

  startTimeout();
  while ((ReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = ReadReg16Bit(RESULT_RANGE_STATUS + 10);

  WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  RMTT_I2C_BUSY_UNLOCK();
  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t RMTT_TOF::ReadRangeSingleMillimeters(void)
{
  RMTT_I2C_BUSY_LOCK();

  WriteReg(0x80, 0x01);
  WriteReg(0xFF, 0x01);
  WriteReg(0x00, 0x00);
  WriteReg(0x91, stop_variable);
  WriteReg(0x00, 0x01);
  WriteReg(0xFF, 0x00);
  WriteReg(0x80, 0x00);

  WriteReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (ReadReg(SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }
  RMTT_I2C_BUSY_UNLOCK();
  return ReadRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool RMTT_TOF::TimeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool RMTT_TOF::GetSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  WriteReg(0x80, 0x01);
  WriteReg(0xFF, 0x01);
  WriteReg(0x00, 0x00);

  WriteReg(0xFF, 0x06);
  WriteReg(0x83, ReadReg(0x83) | 0x04);
  WriteReg(0xFF, 0x07);
  WriteReg(0x81, 0x01);

  WriteReg(0x80, 0x01);

  WriteReg(0x94, 0x6b);
  WriteReg(0x83, 0x00);
  startTimeout();
  while (ReadReg(0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
  WriteReg(0x83, 0x01);
  tmp = ReadReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  WriteReg(0x81, 0x00);
  WriteReg(0xFF, 0x06);
  WriteReg(0x83, ReadReg(0x83)  & ~0x04);
  WriteReg(0xFF, 0x01);
  WriteReg(0x00, 0x01);

  WriteReg(0xFF, 0x00);
  WriteReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void RMTT_TOF::GetSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = ReadReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void RMTT_TOF::GetSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = GetVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = ReadReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    TimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    DecodeTimeout(ReadReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    TimeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = GetVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    DecodeTimeout(ReadReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    TimeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t RMTT_TOF::DecodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
uint16_t RMTT_TOF::EncodeTimeout(uint32_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t RMTT_TOF::TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t RMTT_TOF::TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool RMTT_TOF::PerformSingleRefCalibration(uint8_t vhv_init_byte)
{
  WriteReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((ReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return false; }
  }

  WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  WriteReg(SYSRANGE_START, 0x00);

  return true;
}
