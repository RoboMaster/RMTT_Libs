
// This file is copied from github.
// https://github.com/kkostyan/is31fl3733.git

#pragma once

#include "stdint.h"

// #define __RMTT_MATRIX_IIC_DEBUG__

#define RMTT_MATRIX_CS IS31FL3733_CS
#define RMTT_MATRIX_SW IS31FL3733_SW
#define RMTT_MATRIX_LED_ON  IS31FL3733_LED_STATE_ON
#define RMTT_MATRIX_LED_OFF IS31FL3733_LED_STATE_OFF

/** Number of CS lines.
  */
#define IS31FL3733_CS (16)

/** Number of SW lines.
  */
#define IS31FL3733_SW (8)

/** IS31FL3733 base address on I2C bus.
  */
#define IS31FL3733_I2C_BASE_ADDR (0xA0)
#define IS31FL3733_I2C_7_BIT_ADDR (0x50)
/** IS31FL3733 ADDR[2:1] connection.
  */
#define ADDR_GND (0x00) ///< ADDRx pin connected to GND.
#define ADDR_SCL (0x01) ///< ADDRx pin connected to SCL.
#define ADDR_SDA (0x02) ///< ADDRx pin connected to SDA.
#define ADDR_VCC (0x03) ///< ADDRx pin connected to VCC.

/** IS31FL3733 real address on I2C bus, see Table 1 on page 9 in datasheet.
    Example: IS31FL3733_I2C_ADDR(ADDR_SDA, ADDR_VCC) is 0xB6 address on I2C bus.
  */
#define IS31FL3733_I2C_ADDR(ADDR2, ADDR1) (uint8_t)((IS31FL3733_I2C_BASE_ADDR) | ((ADDR2) << 3) | ((ADDR1) << 1))

/** IS31FL3733 common registers.
  */
#define IS31FL3733_PSR  (0xFD) ///< Page select register. Write only.
#define IS31FL3733_PSWL (0xFE) ///< Page select register write lock. Read/Write.
#define IS31FL3733_IMR  (0xF0) ///< Interrupt mask register. Write only.
#define IS31FL3733_ISR  (0xF1) ///< Interrupt status register. Read only.

/** Registers in Page 0.
  */
#define IS31FL3733_LEDONOFF (0x0000) /// ON or OFF state control for each LED. Write only.
#define IS31FL3733_LEDOPEN  (0x0018) /// Open state for each LED. Read only.
#define IS31FL3733_LEDSHORT (0x0030) /// Short state for each LED. Read only.

/** Registers in Page 1.
  */
#define IS31FL3733_LEDPWM (0x0100) /// PWM duty for each LED. Write only.

/** Registers in Page 2.
  */
#define IS31FL3733_LEDABM (0x0200) /// Auto breath mode for each LED. Write only.

/** Registers in Page 3.
  */
#define IS31FL3733_CR    (0x0300) /// Configuration Register. Write only.
#define IS31FL3733_GCC   (0x0301) /// Global Current Control register. Write only.
#define IS31FL3733_ABM1  (0x0302) /// Auto breath control register for ABM-1. Write only.
#define IS31FL3733_ABM2  (0x0306) /// Auto breath control register for ABM-2. Write only.
#define IS31FL3733_ABM3  (0x030A) /// Auto breath control register for ABM-3. Write only.
#define IS31FL3733_TUR   (0x030E) /// Time update register. Write only.
#define IS31FL3733_SWPUR (0x030F) /// SWy Pull-Up Resistor selection register. Write only.
#define IS31FL3733_CSPDR (0x0310) /// CSx Pull-Down Resistor selection register. Write only.
#define IS31FL3733_RESET (0x0311) /// Reset register. Read only.

/// Get register page.
#define IS31FL3733_GET_PAGE(reg_addr) (uint8_t)((reg_addr) >> 8)
/// Get register 8-bit address.
#define IS31FL3733_GET_ADDR(reg_addr) (uint8_t)(reg_addr)

/// PSWL register bits.
#define IS31FL3733_PSWL_DISABLE (0x00) /// Disable write to Page Select register.
#define IS31FL3733_PSWL_ENABLE  (0xC5) /// Enable write to Page select register.

/// IMR register bits.
#define IS31FL3733_IMR_IAC (0x08) /// Auto Clear Interrupt bit.
#define IS31FL3733_IMR_IAB (0x04) /// Auto Breath Interrupt bit.
#define IS31FL3733_IMR_IS  (0x02) /// Dot Short Interrupt bit.
#define IS31FL3733_IMR_IO  (0x01) /// Dot Open Interrupt bit.

/// ISR register bits.
#define IS31FL3733_ISR_ABM3 (0x10) /// Auto Breath Mode 3 Finish Bit.
#define IS31FL3733_ISR_ABM2 (0x08) /// Auto Breath Mode 2 Finish Bit.
#define IS31FL3733_ISR_ABM1 (0x04) /// Auto Breath Mode 1 Finish Bit.
#define IS31FL3733_ISR_SB   (0x02) /// Short Bit.
#define IS31FL3733_ISR_OB   (0x01) /// Open Bit.

/// CR register bits.
#define IS31FL3733_CR_SYNC_MASTER (0x40) /// Configure as clock master device.
#define IS31FL3733_CR_SYNC_SLAVE  (0x80) /// Configure as clock slave device.
#define IS31FL3733_CR_OSD         (0x04) /// Open/Short detection enable bit.
#define IS31FL3733_CR_BEN         (0x02) /// Auto breath mode enable bit.
#define IS31FL3733_CR_SSD         (0x01) /// Software shutdown bit.

/// Maximum number of ABM loop times.
#define IS31FL3733_ABM_LOOP_TIMES_MAX (0x0FFF)
/// Enter to ABM endless loop.
#define IS31FL3733_ABM_LOOP_FOREVER (0x0000)

/// LED state enumeration.
typedef enum {
  IS31FL3733_LED_STATE_OFF = 0x00, ///< LED is off.
  IS31FL3733_LED_STATE_ON  = 0x01  ///< LED is on.
} IS31FL3733_LED_STATE;

/// LED status enumeration.
typedef enum {
  IS31FL3733_LED_STATUS_NORMAL  = 0x00, ///< Normal LED status.
  IS31FL3733_LED_STATUS_OPEN    = 0x01, ///< LED is open.
  IS31FL3733_LED_STATUS_SHORT   = 0x02, ///< LED is short.
  IS31FL3733_LED_STATUS_UNKNOWN = 0x03  ///< Unknown LED status.
} IS31FL3733_LED_STATUS;

/// Pull-Up or Pull-Down resistor value.
typedef enum {
  IS31FL3733_RESISTOR_OFF = 0x00, ///< No resistor.
  IS31FL3733_RESISTOR_500 = 0x01, ///< 0.5 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_1K  = 0x02, ///< 1.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_2K  = 0x03, ///< 2.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_4K  = 0x04, ///< 4.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_8K  = 0x05, ///< 8.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_16K = 0x06, ///< 16 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_32K = 0x07  ///< 32 kOhm pull-up resistor.
} IS31FL3733_RESISTOR;

/// LED mode enumeration.
typedef enum {
  IS31FL3733_LED_MODE_PWM  = 0x00, ///< PWM control mode.
  IS31FL3733_LED_MODE_ABM1 = 0x01, ///< Auto Breath Mode 1.
  IS31FL3733_LED_MODE_ABM2 = 0x02, ///< Auto Breath Mode 2.
  IS31FL3733_LED_MODE_ABM3 = 0x03  ///< Auto Breath Mode 3.
} IS31FL3733_LED_MODE;

/// ABM T1 period time, ms.
typedef enum {
  IS31FL3733_ABM_T1_210MS   = 0x00,
  IS31FL3733_ABM_T1_420MS   = 0x20,
  IS31FL3733_ABM_T1_840MS   = 0x40,
  IS31FL3733_ABM_T1_1680MS  = 0x60,
  IS31FL3733_ABM_T1_3360MS  = 0x80,
  IS31FL3733_ABM_T1_6720MS  = 0xA0,
  IS31FL3733_ABM_T1_13440MS = 0xC0,
  IS31FL3733_ABM_T1_26880MS = 0xE0
} IS31FL3733_ABM_T1;

/// ABM T2 period time, ms.
typedef enum {
  IS31FL3733_ABM_T2_0MS     = 0x00,
  IS31FL3733_ABM_T2_210MS   = 0x02,
  IS31FL3733_ABM_T2_420MS   = 0x04,
  IS31FL3733_ABM_T2_840MS   = 0x06,
  IS31FL3733_ABM_T2_1680MS  = 0x08,
  IS31FL3733_ABM_T2_3360MS  = 0x0A,
  IS31FL3733_ABM_T2_6720MS  = 0x0C,
  IS31FL3733_ABM_T2_13440MS = 0x0E,
  IS31FL3733_ABM_T2_26880MS = 0x10
} IS31FL3733_ABM_T2;

/// ABM T3 period time, ms.
typedef enum {
  IS31FL3733_ABM_T3_210MS   = 0x00,
  IS31FL3733_ABM_T3_420MS   = 0x20,
  IS31FL3733_ABM_T3_840MS   = 0x40,
  IS31FL3733_ABM_T3_1680MS  = 0x60,
  IS31FL3733_ABM_T3_3360MS  = 0x80,
  IS31FL3733_ABM_T3_6720MS  = 0xA0,
  IS31FL3733_ABM_T3_13440MS = 0xC0,
  IS31FL3733_ABM_T3_26880MS = 0xE0
} IS31FL3733_ABM_T3;

/// ABM T4 period time, ms.
typedef enum {
  IS31FL3733_ABM_T4_0MS      = 0x00,
  IS31FL3733_ABM_T4_210MS    = 0x02,
  IS31FL3733_ABM_T4_420MS    = 0x04,
  IS31FL3733_ABM_T4_840MS    = 0x06,
  IS31FL3733_ABM_T4_1680MS   = 0x08,
  IS31FL3733_ABM_T4_3360MS   = 0x0A,
  IS31FL3733_ABM_T4_6720MS   = 0x0C,
  IS31FL3733_ABM_T4_13440MS  = 0x0E,
  IS31FL3733_ABM_T4_26880MS  = 0x10,
  IS31FL3733_ABM_T4_53760MS  = 0x12,
  IS31FL3733_ABM_T4_107520MS = 0x14
} IS31FL3733_ABM_T4;

/// ABM loop beginning time.
typedef enum {
  IS31FL3733_ABM_LOOP_BEGIN_T1 = 0x00, ///< Loop begin from T1.
  IS31FL3733_ABM_LOOP_BEGIN_T2 = 0x10, ///< Loop begin from T2.
  IS31FL3733_ABM_LOOP_BEGIN_T3 = 0x20, ///< Loop begin from T3.
  IS31FL3733_ABM_LOOP_BEGIN_T4 = 0x30  ///< Loop begin from T4.
} IS31FL3733_ABM_LOOP_BEGIN;

/// ABM loop end time.
typedef enum {
  IS31FL3733_ABM_LOOP_END_T3 = 0x00, ///< Loop end at end of T3.
  IS31FL3733_ABM_LOOP_END_T1 = 0x40  ///< Loop end at end of T1.
} IS31FL3733_ABM_LOOP_END;

/// ABM function number (also used as register offset).
typedef enum {
  IS31FL3733_ABM_NUM_1 = IS31FL3733_ABM1,
  IS31FL3733_ABM_NUM_2 = IS31FL3733_ABM2,
  IS31FL3733_ABM_NUM_3 = IS31FL3733_ABM3
} IS31FL3733_ABM_NUM;

/** Auto Breath Mode (ABM) configuration structure.
  *      +----+              +
  *     /      \            /
  *    /        \          /
  *   /          \        /
  *  /            \      /
  * +              +----+
  * | T1 | T2 | T3 | T4 | T1 |
  *
  */
typedef struct {
  /// T1 time.
  IS31FL3733_ABM_T1 T1;
  /// T2 time.
  IS31FL3733_ABM_T2 T2;
  /// T3 time.
  IS31FL3733_ABM_T3 T3;
  /// T4 time.
  IS31FL3733_ABM_T4 T4;
  /// Loop beginning time.
  IS31FL3733_ABM_LOOP_BEGIN Tbegin;
  /// Loop end time.
  IS31FL3733_ABM_LOOP_END Tend;
  /// Total loop times.
  uint16_t Times;
} IS31FL3733_ABM;

/** IS31FL3733 structure.
  */
typedef struct {
  /// Address on I2C bus.
  uint8_t address;
  /// State of individual LED's. Bitmask, that can't be read back from IS31FL3733.
  uint8_t leds[IS31FL3733_SW * IS31FL3733_CS / 8];
  /// Pointer to I2C write register function.
  uint8_t (*i2c_write_reg) (uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count);
  /// Pointer to I2C read register function.
  uint8_t (*i2c_read_reg) (uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count);
} IS31FL3733;

class RMTT_Matrix
{
private:
public:
    RMTT_Matrix(){};
    ~RMTT_Matrix(){};

    static void On();
    static void Off();

    static void Init(uint8_t gcc);
    static void SetGCC(uint8_t gcc);
    static void SetLEDStatus(uint8_t cs, uint8_t sw, IS31FL3733_LED_STATE state);
    static void SetLEDPWM(uint8_t cs, uint8_t sw, uint8_t value);
    static void SetAllPWM(uint8_t *val);

    static uint8_t ReadCommonReg(uint8_t reg_addr);
    static void WriteCommonReg(uint8_t reg_addr, uint8_t reg_value);
    static void SetLEDMode(uint8_t cs, uint8_t sw, IS31FL3733_LED_MODE mode);
    static void ConfigABM(IS31FL3733_ABM_NUM n, IS31FL3733_ABM *config);
    static void StartABM();
};

/// Read from common register.
uint8_t IS31FL3733_ReadCommonReg (IS31FL3733 *device, uint8_t reg_addr);
/// Write to common register.
void IS31FL3733_WriteCommonReg (IS31FL3733 *device, uint8_t reg_addr, uint8_t reg_value);
/// Select active page.
void IS31FL3733_SelectPage (IS31FL3733 *device, uint8_t page);
/// Read from paged register.
uint8_t IS31FL3733_ReadPagedReg (IS31FL3733 *device, uint16_t reg_addr);
/// Write to paged register.
void IS31FL3733_WritePagedReg (IS31FL3733 *device, uint16_t reg_addr, uint8_t reg_value);
/// Write array to sequentially allocated paged registers starting from specified address.
void IS31FL3733_WritePagedRegs (IS31FL3733 *device, uint16_t reg_addr, uint8_t *values, uint8_t count);
/// Initialize IS31FL3733 for PWM operation.
void IS31FL3733_Init (IS31FL3733 *device);
/// Set global current control register.
void IS31FL3733_SetGCC (IS31FL3733 *device, uint8_t gcc);
/// Set SW Pull-Up register.
void IS31FL3733_SetSWPUR (IS31FL3733 *device, IS31FL3733_RESISTOR resistor);
/// Set CS Pull-Down register.
void IS31FL3733_SetCSPDR (IS31FL3733 *device, IS31FL3733_RESISTOR resistor);
/// Set LED state: ON/OFF. Could be set ALL / CS / SW.
void IS31FL3733_SetLEDState (IS31FL3733 *device, uint8_t cs, uint8_t sw, IS31FL3733_LED_STATE state);
/// Set LED PWM duty value. Could be set ALL / CS / SW.
void IS31FL3733_SetLEDPWM (IS31FL3733 *device, uint8_t cs, uint8_t sw, uint8_t value);
/// Get status of LED.
IS31FL3733_LED_STATUS IS31FL3733_GetLEDStatus (IS31FL3733 *device, uint8_t cs, uint8_t sw);
/// Set LED state for all LED's from buffer.
void IS31FL3733_SetState (IS31FL3733 *device, uint8_t *states);
/// SET LED PWM duty value for all LED's from buffer.
void IS31FL3733_SetPWM (IS31FL3733 *device, uint8_t *values);
/// Set LED operating mode: PWM/ABM1,2,3. Could be set ALL / CS / SW.
void IS31FL3733_SetLEDMode (IS31FL3733 *device, uint8_t cs, uint8_t sw, IS31FL3733_LED_MODE mode);
/// Configure ABM Mode.
void IS31FL3733_ConfigABM (IS31FL3733 *device, IS31FL3733_ABM_NUM n, IS31FL3733_ABM *config);
/// Start ABM operation.
void IS31FL3733_StartABM (IS31FL3733 *device);
