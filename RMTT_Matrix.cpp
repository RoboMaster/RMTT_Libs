
#include <Arduino.h>
#include <Wire.h>
#include "RMTT_Libs.h"

// This file is copied from github.
// https://github.com/kkostyan/is31fl3733.git

static IS31FL3733 LEDMatrix;
uint8_t I2CWriteMulti(uint8_t address, uint8_t reg, uint8_t *src, uint8_t count);
uint8_t I2CReadMulti(uint8_t address, uint8_t reg, uint8_t *dst, uint8_t count);

void RMTT_Matrix::Init(uint8_t gcc)
{
    LEDMatrix.address = 0x50;
    LEDMatrix.i2c_write_reg = I2CWriteMulti;
    LEDMatrix.i2c_read_reg = I2CReadMulti;
#ifdef __RMTT_MATRIX_IIC_DEBUG__
    Serial.printf("Ardress 0x%x  \r\n", LEDMatrix.address);
#endif
    IS31FL3733_Init(&LEDMatrix);
    // Set Global Current Control.
    IS31FL3733_SetGCC(&LEDMatrix, gcc);

    IS31FL3733_SetLEDState (&LEDMatrix, IS31FL3733_CS, IS31FL3733_SW, IS31FL3733_LED_STATE_ON);
}

void RMTT_Matrix::SetGCC(uint8_t gcc)
{
    RMTT_I2C_BUSY_LOCK();
    // Set Global Current Control.
    IS31FL3733_SetGCC(&LEDMatrix, gcc);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::SetLEDStatus(uint8_t cs, uint8_t sw, IS31FL3733_LED_STATE state)
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_SetLEDState (&LEDMatrix, cs, sw, state);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::SetLEDPWM(uint8_t cs, uint8_t sw, uint8_t value)
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_SetLEDPWM (&LEDMatrix, cs, sw, value);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::SetAllPWM(uint8_t *val)
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_SetPWM (&LEDMatrix, (uint8_t*)val);
    RMTT_I2C_BUSY_UNLOCK();
}

uint8_t RMTT_Matrix::ReadCommonReg (uint8_t reg_addr)
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_ReadCommonReg (&LEDMatrix, reg_addr);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::WriteCommonReg (uint8_t reg_addr, uint8_t reg_value)
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_WriteCommonReg (&LEDMatrix, reg_addr, reg_value);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::SetLEDMode (uint8_t cs, uint8_t sw, IS31FL3733_LED_MODE mode)
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_SetLEDMode (&LEDMatrix, cs, sw, mode);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::ConfigABM (IS31FL3733_ABM_NUM n, IS31FL3733_ABM *config)
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_ConfigABM (&LEDMatrix, n, config);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::StartABM ()
{
    RMTT_I2C_BUSY_LOCK();
    IS31FL3733_StartABM (&LEDMatrix);
    RMTT_I2C_BUSY_UNLOCK();
}

void RMTT_Matrix::On()
{
   pinMode(5,OUTPUT);
   digitalWrite(5, HIGH);
}

void RMTT_Matrix::Off()
{
   pinMode(5,OUTPUT);
   digitalWrite(5, LOW);
}

/**************************IS31FL3733 C Driver***********************************************/

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
uint8_t I2CWriteMulti(uint8_t address, uint8_t reg, uint8_t *src, uint8_t count)
{
    uint8_t I2C_status = 0;

#ifdef __RMTT_MATRIX_IIC_DEBUG__
    Serial.printf("I2C Write Ardress 0x%x Reg: 0x%x count: 0x%x\r\n", address, reg, count);
#endif

    Wire.beginTransmission(address);
    Wire.write(reg);

    while (count-- > 0)
    {
#ifdef __RMTT_MATRIX_IIC_DEBUG__
       Serial.printf("0x%x ", *src);
#endif
       Wire.write(*(src++));
    }

#ifdef __RMTT_MATRIX_IIC_DEBUG__
    Serial.println();
#endif

    I2C_status = Wire.endTransmission();
#ifdef __RMTT_MATRIX_IIC_DEBUG__
    Serial.printf("I2C_status %d  \r\n", I2C_status);
#endif
    return I2C_status;
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
uint8_t I2CReadMulti(uint8_t address, uint8_t reg, uint8_t *dst, uint8_t count)
{
    uint8_t I2C_status = 0;

#ifdef __RMTT_MATRIX_IIC_DEBUG__
    uint8_t read_len = count;
    uint8_t *read_ptr = dst;

    Serial.printf("I2C Read Ardress 0x%x  Reg: 0x%x count: 0x%x\r\n", address, reg, count);
#endif

    Wire.beginTransmission(address);
    Wire.write(reg);
    I2C_status = Wire.endTransmission();

    Wire.requestFrom(address, count);

    while (count-- > 0)
    {
        *(dst++) = Wire.read();
    }
#ifdef __RMTT_MATRIX_IIC_DEBUG__
    for(int i = 0; i < read_len; i++)
    {
      Serial.printf("0x%x ", read_ptr[i]);
    }
    Serial.println();
    Serial.printf("I2C_status %d  \r\n", I2C_status);
#endif

    return I2C_status;
}

uint8_t IS31FL3733_ReadCommonReg (IS31FL3733 *device, uint8_t reg_addr)
{
  uint8_t reg_value;

  // Read value from register.
  device->i2c_read_reg (device->address, reg_addr, &reg_value, sizeof(uint8_t));
  // Return register value.
  return reg_value;
}

void IS31FL3733_WriteCommonReg (IS31FL3733 *device, uint8_t reg_addr, uint8_t reg_value)
{
  // Write value to register.
  device->i2c_write_reg (device->address, reg_addr, &reg_value, sizeof(uint8_t));
}

void IS31FL3733_SelectPage (IS31FL3733 *device, uint8_t page)
{
  // Unlock Command Register.
  IS31FL3733_WriteCommonReg (device, IS31FL3733_PSWL, IS31FL3733_PSWL_ENABLE);
  // Select requested page in Command Register.
  IS31FL3733_WriteCommonReg (device, IS31FL3733_PSR, page);
}

uint8_t IS31FL3733_ReadPagedReg (IS31FL3733 *device, uint16_t reg_addr)
{
  uint8_t reg_value;

  // Select register page.
  IS31FL3733_SelectPage (device, IS31FL3733_GET_PAGE(reg_addr));
  // Read value from register.
  device->i2c_read_reg (device->address, IS31FL3733_GET_ADDR(reg_addr), &reg_value, sizeof(uint8_t));
  // Return register value.
  return reg_value;
}

void IS31FL3733_WritePagedReg (IS31FL3733 *device, uint16_t reg_addr, uint8_t reg_value)
{
  // Select register page.
  IS31FL3733_SelectPage (device, IS31FL3733_GET_PAGE(reg_addr));
  // Write value to register.
  device->i2c_write_reg (device->address, IS31FL3733_GET_ADDR(reg_addr), &reg_value, sizeof(uint8_t));
}

void IS31FL3733_WritePagedRegs (IS31FL3733 *device, uint16_t reg_addr, uint8_t *values, uint8_t count)
{
  // Select registers page.
  IS31FL3733_SelectPage (device, IS31FL3733_GET_PAGE(reg_addr));
  // Write values to registers.
  device->i2c_write_reg (device->address, IS31FL3733_GET_ADDR(reg_addr), values, count);
}

void IS31FL3733_Init (IS31FL3733 *device)
{
  // Read reset register to reset device.
  IS31FL3733_ReadPagedReg (device, IS31FL3733_RESET);
  // Clear software reset in configuration register.
  IS31FL3733_WritePagedReg (device, IS31FL3733_CR, IS31FL3733_CR_SSD);
  // Clear state of all LEDs in internal buffer and sync buffer to device.
  IS31FL3733_SetLEDState (device, IS31FL3733_CS, IS31FL3733_SW, IS31FL3733_LED_STATE_OFF);
}

void IS31FL3733_SetGCC (IS31FL3733 *device, uint8_t gcc)
{
  // Write gcc value to Global Current Control (GCC) register.
  IS31FL3733_WritePagedReg (device, IS31FL3733_GCC, gcc);
}

void IS31FL3733_SetSWPUR (IS31FL3733 *device, IS31FL3733_RESISTOR resistor)
{
  // Write resistor value to SWPUR register.
  IS31FL3733_WritePagedReg (device, IS31FL3733_SWPUR, resistor);
}

void IS31FL3733_SetCSPDR (IS31FL3733 *device, IS31FL3733_RESISTOR resistor)
{
  // Write resistor value to CSPDR register.
  IS31FL3733_WritePagedReg (device, IS31FL3733_CSPDR, resistor);
}

void IS31FL3733_SetLEDState (IS31FL3733 *device, uint8_t cs, uint8_t sw, IS31FL3733_LED_STATE state)
{
  uint8_t offset;

  // Check SW boundaries.
  if (sw < IS31FL3733_SW)
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set state of individual LED.
      // Calculate LED bit offset.
      offset = (sw << 1) + (cs / 8);
      // Update state of LED in internal buffer.
      if (state == IS31FL3733_LED_STATE_OFF)
      {
        // Clear bit for selected LED.
        device->leds[offset] &= ~(0x01 << (cs % 8));
      }
      else
      {
        // Set bit for selected LED.
        device->leds[offset] |= 0x01 << (cs % 8);
      }
      // Write updated LED state to device register.
      IS31FL3733_WritePagedReg (device, IS31FL3733_LEDONOFF + offset, device->leds[offset]);
    }
    else
    {
      // Set state of full row selected by SW.
      // Calculate row offset.
      offset = sw << 1;
      // Update state of row LEDs in internal buffer.
      if (state == IS31FL3733_LED_STATE_OFF)
      {
        // Clear 16 bits for selected row LEDs.
        device->leds[offset    ] = 0x00;
        device->leds[offset + 1] = 0x00;
      }
      else
      {
        // Set 16 bits for selected row LEDs.
        device->leds[offset    ] = 0xFF;
        device->leds[offset + 1] = 0xFF;
      }
      // Write updated LEDs state to device registers.
      IS31FL3733_WritePagedRegs (device, IS31FL3733_LEDONOFF + offset, &device->leds[offset], IS31FL3733_CS / 8);
    }
  }
  else
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set state of full column selected by CS.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        // Calculate LED bit offset.
        offset = (sw << 1) + (cs / 8);
        // Update state of LED in internal buffer.
        if (state == IS31FL3733_LED_STATE_OFF)
        {
          // Clear bit for selected LED.
          device->leds[offset] &= ~(0x01 << (cs % 8));
        }
        else
        {
          // Set bit for selected LED.
          device->leds[offset] |= 0x01 << (cs % 8);
        }
        // Write updated LED state to device register.
        IS31FL3733_WritePagedReg (device, IS31FL3733_LEDONOFF + offset, device->leds[offset]);
      }
    }
    else
    {
      // Set state of all LEDs.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        // Update state of all LEDs in internal buffer.
        if (state == IS31FL3733_LED_STATE_OFF)
        {
          // Clear all bits.
          device->leds[(sw << 1)    ] = 0x00;
          device->leds[(sw << 1) + 1] = 0x00;
        }
        else
        {
          // Set all bits.
          device->leds[(sw << 1)    ] = 0xFF;
          device->leds[(sw << 1) + 1] = 0xFF;
        }
      }
      // Write updated LEDs state to device registers.
      IS31FL3733_WritePagedRegs (device, IS31FL3733_LEDONOFF, device->leds, IS31FL3733_SW * IS31FL3733_CS / 8);
    }
  }
}

void IS31FL3733_SetLEDPWM (IS31FL3733 *device, uint8_t cs, uint8_t sw, uint8_t value)
{
  uint8_t offset;

  // Check SW boundaries.
  if (sw < IS31FL3733_SW)
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set PWM of individual LED.
      // Calculate LED offset.
      offset = sw * IS31FL3733_CS + cs;
      // Write LED PWM value to device register.
      IS31FL3733_WritePagedReg (device, IS31FL3733_LEDPWM + offset, value);
    }
    else
    {
      // Set PWM of full row selected by SW.
      for (cs = 0; cs < IS31FL3733_CS; cs++)
      {
        // Calculate LED offset.
        offset = sw * IS31FL3733_CS + cs;
        // Write LED PWM value to device register.
        IS31FL3733_WritePagedReg (device, IS31FL3733_LEDPWM + offset, value);
      }
    }
  }
  else
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set PWM of full column selected by CS.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
          // Calculate LED offset.
          offset = sw * IS31FL3733_CS + cs;
          // Write LED PWM value to device register.
          IS31FL3733_WritePagedReg (device, IS31FL3733_LEDPWM + offset, value);
      }
    }
    else
    {
      // Set PWM of all LEDs.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        for (cs = 0; cs < IS31FL3733_CS; cs++)
        {
          // Calculate LED offset.
          offset = sw * IS31FL3733_CS + cs;
          // Write LED PWM value to device register.
          IS31FL3733_WritePagedReg (device, IS31FL3733_LEDPWM + offset, value);
        }
      }
    }
  }
}

IS31FL3733_LED_STATUS IS31FL3733_GetLEDStatus (IS31FL3733 *device, uint8_t cs, uint8_t sw)
{
  uint8_t offset;

  // Check CS and SW boundaries.
  if ((cs < IS31FL3733_CS) && (sw < IS31FL3733_SW))
  {
    // Calculate LED bit offset.
    offset = (sw << 1) + (cs / 8);
    // Get Open status from device register.
    if (IS31FL3733_ReadPagedReg (device, IS31FL3733_LEDOPEN + offset) & (0x01 << (cs % 8)))
    {
      return IS31FL3733_LED_STATUS_OPEN;
    }
    // Get Short status from device register.
    if (IS31FL3733_ReadPagedReg (device, IS31FL3733_LEDSHORT + offset) & (0x01 << (cs % 8)))
    {
      return IS31FL3733_LED_STATUS_SHORT;
    }
  }
  else
  {
    // Unknown status for nonexistent LED.
    return IS31FL3733_LED_STATUS_UNKNOWN;
  }
  return IS31FL3733_LED_STATUS_NORMAL;
}

void IS31FL3733_SetState (IS31FL3733 *device, uint8_t *states)
{
  uint8_t sw;
  uint8_t cs;
  uint8_t offset;

  // Set state of all LEDs.
  for (sw = 0; sw < IS31FL3733_SW; sw++)
  {
    for (cs = 0; cs < IS31FL3733_CS; cs++)
    {
      // Calculate LED bit offset.
      offset = (sw << 1) + (cs / 8);
      // Update state of LED in internal buffer.
      if (states[sw * IS31FL3733_CS + cs] == 0)
      {
        // Clear bit for selected LED.
        device->leds[offset] &= ~(0x01 << (cs % 8));
      }
      else
      {
        // Set bit for selected LED.
        device->leds[offset] |= 0x01 << (cs % 8);
      }
    }
  }
  // Write updated LEDs state to device registers.
  IS31FL3733_WritePagedRegs (device, IS31FL3733_LEDONOFF, device->leds, IS31FL3733_SW * IS31FL3733_CS / 8);
}

void IS31FL3733_SetPWM (IS31FL3733 *device, uint8_t *values)
{
  // Write LED PWM values to device registers.
  IS31FL3733_WritePagedRegs (device, IS31FL3733_LEDPWM, values, IS31FL3733_SW * IS31FL3733_CS / 2);
  IS31FL3733_WritePagedRegs (device, IS31FL3733_LEDPWM + IS31FL3733_SW * IS31FL3733_CS / 2, values + IS31FL3733_SW * IS31FL3733_CS / 2, IS31FL3733_SW * IS31FL3733_CS / 2);
}

void IS31FL3733_SetLEDMode (IS31FL3733 *device, uint8_t cs, uint8_t sw, IS31FL3733_LED_MODE mode)
{
  uint8_t offset;

  // Check SW boundaries.
  if (sw < IS31FL3733_SW)
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set mode of individual LED.
      // Calculate LED offset.
      offset = sw * IS31FL3733_CS + cs;
      // Write LED mode to device register.
      IS31FL3733_WritePagedReg (device, IS31FL3733_LEDABM + offset, mode);
    }
    else
    {
      // Set mode of full row selected by SW.
      for (cs = 0; cs < IS31FL3733_CS; cs++)
      {
        // Calculate LED offset.
        offset = sw * IS31FL3733_CS + cs;
        // Write LED mode to device register.
        IS31FL3733_WritePagedReg (device, IS31FL3733_LEDABM + offset, mode);
      }
    }
  }
  else
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set mode of full column selected by CS.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
          // Calculate LED offset.
          offset = sw * IS31FL3733_CS + cs;
          // Write LED mode to device register.
          IS31FL3733_WritePagedReg (device, IS31FL3733_LEDABM + offset, mode);
      }
    }
    else
    {
      // Set mode of all LEDs.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        for (cs = 0; cs < IS31FL3733_CS; cs++)
        {
          // Calculate LED offset.
          offset = sw * IS31FL3733_CS + cs;
          // Write LED mode to device register.
          IS31FL3733_WritePagedReg (device, IS31FL3733_LEDABM + offset, mode);
        }
      }
    }
  }
}

void IS31FL3733_ConfigABM (IS31FL3733 *device, IS31FL3733_ABM_NUM n, IS31FL3733_ABM *config)
{
  // Set fade in and fade out time.
  IS31FL3733_WritePagedReg (device, n, config->T1 | config->T2);
  // Set hold and off time.
  IS31FL3733_WritePagedReg (device, n + 1, config->T3 | config->T4);
  // Set loop begin/end time and high part of loop times.
  IS31FL3733_WritePagedReg (device, n + 2, config->Tend | config->Tbegin | ((config->Times >> 8) & 0x0F));
  // Set low part of loop times.
  IS31FL3733_WritePagedReg (device, n + 3, config->Times & 0xFF);
}

void IS31FL3733_StartABM (IS31FL3733 *device)
{
  // Clear B_EN bit in configuration register.
  IS31FL3733_WritePagedReg (device, IS31FL3733_CR, IS31FL3733_CR_SSD);
  // Set B_EN bit in configuration register.
  IS31FL3733_WritePagedReg (device, IS31FL3733_CR, IS31FL3733_CR_BEN | IS31FL3733_CR_SSD);
  // Write 0x00 to Time Update Register to update ABM settings.
  IS31FL3733_WritePagedReg (device, IS31FL3733_TUR, 0x00);
}
