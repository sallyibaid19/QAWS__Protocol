#include "drv_lib.h"
#include "drv_reg_map.h"
#include "mcc_generated_files/mcc.h"

#define _XTAL_FRQ    4000000UL

volatile DRV8434S_REG_t getDriverReg;

/*
 Sleep mode. 
 * Logic high to enable device; 
 * logic low to enter lowpower sleep mode; 
 * An nSLEEP low pulse clears faults.
 */
/* Device Parameters */
volatile uint8_t nSleep_Pin; // 0: Device Sleep , 1: Device Awake 
volatile uint8_t nFault_Pin; // 0: No fault , 1: Fault 


/* Device Parameters*/
volatile uint8_t nSleepFlag; /**< 0: Device Sleep , 1: Device Awake */
volatile uint8_t gDRVENFlag; /**< 0: Outputs Hi-Z, 1: Outputs active, */
volatile uint8_t nFaultFlag; /**< 0: No fault , 1: Fault */
volatile uint8_t ClrFltFlag;

//configures the DRV8434S PINs

void drv8434s_init(void) {
    //TODO -------NEED TO CHANGE
    // All OUTPUT >> nSLEEP, DIR, STEP, ENABLE
    nSLEEP = 0;
    ENABLE = 0;
    DIR = 0;
    STEP = 0;
    //nSLEEP low, DIR low, STEP low , ENABLE high
    SLEEP_low(); //sleep mode
    DIR_low();
    STEP_low();
    ENABLE_high();
}

/*
 * (SPI) Communication
 * full duplex, 
 * 4-wire synchronous communication. 
 * >>>> ? One target device
 * SPI Format
 * The SDI input data word is 16 bits long,
 *  and consists of the following format:
 *         ? 1 read or write bit, W (bit 14)
 *         ? 5 address bits, A (bits 13 through 9)
 *         ? 8 data bits, D (bits 7 through 0)
 */

/*
 * ==================================================================================================
 * Register format | 0 | R/W | A4 | A3 | A2 | A1 | A0 | x | D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 |
 * Ax : address bit, 
 * Dx : data bits and 
 * R/W is read write bit.
 * For read R/W bit should be 1.
 * ==================================================================================================
 */


uint16_t Read_Register(uint8_t address) {
    volatile uint16_t reg_value = 0;
    volatile uint8_t MSB = 0;
    volatile uint8_t LSB = 0;
    reg_value |= READ_OPERATION; // Set R/W bit
    reg_value |= ((address << 9) & 0x3F00); // Configure register address value
    CS = 0; // SPI set 
    SPI1TXB = (uint8_t) ((reg_value >> 8) & 0xFF); // Transmit the Address(MSB Byte)
    while (!PIR3bits.SPI1RXIF);
    MSB = SPI1RXB; // Recieve the First byte of data, MSB byte
    SPI1TXB = 0x00;
    while (!PIR3bits.SPI1RXIF);
    LSB = SPI1RXB; // Recieve the Second byte of data,LSB byte
    SPI1TXB = 0x00;
    CS = 1; // SPI Reset
    __delay_us(10);
    reg_value = ((((MSB << 8) | LSB) & 0x00FF) >> 0x0); // complete data
    return (reg_value);

}

/*
 * ==================================================================================================
 * Register format >>  | 0 | R/W | A4 | A3 | A2 | A1 | A0 | x | D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 |
 * Ax : address bit, 
 * Dx : data bits and 
 * R/W is read write bit.
 * For write R/W bit should 0.
 * ==================================================================================================
 */
uint16_t Write_Register(uint8_t address, uint16_t data) {
    volatile uint16_t reg_value = 0;
    volatile uint8_t MSB = 0;
    volatile uint8_t LSB = 0;
    reg_value |= ((address << 9) & 0x3F00); // Adding register address value
    reg_value |= ((data << 0x0) & 0x00FF); // Adding data value  
    CS = 0; // SPI set 
    MSB = (uint8_t) ((reg_value >> 8) & 0xFF);
    SPI1TXB = MSB; // Transmit first Byte,MSB-Byte
    __delay_us(10);
    LSB = (uint8_t) (reg_value & 0xFF);
    SPI1TXB = LSB; // Transmit Second Byte, LSB-Byte   
    __delay_us(10);
    CS = 1; // SPI Reset
    __delay_us(10);
    return 0;
}

// Changes all of the driver's settings back to their default values.
/*
 call  it at  the beginning of the program 
 to ensure that there are no settings left over from an earlier time 
 * that might affect the operation of the driver.
 */

/*
 default value:
CTRL1 = 0b00000000;  //0x00
CTRL2 = 0b00001111;  //0x0F
CTRL3 = 0b00000110;  //0x06
CTRL4 = 0b00110000;  //0x30
CTRL5 = 0b00001000;  //0x08
CTRL6 = 0b00000011;  //0x03
CTRL7 = 0b00100000;  //0x20

CTRL8 = 0b11111111;  //0xFF [read only]
CTRL9 = 0b00001111;  //0x0F [read only]
 
 
 */
void setDefaultRegisters(void) {

    // Write Control Register Values with default values
    getDriverReg.ctrl1_reg = 0x00; /* CTRL1 register*/
    Write_Register((uint8_t) REG_CTRL1, (uint16_t) getDriverReg.ctrl1_reg);

    getDriverReg.ctrl2_reg = 0x0F; /* CTRL2 register*/
    Write_Register((uint8_t) REG_CTRL2, (uint16_t) getDriverReg.ctrl2_reg);

    getDriverReg.ctrl1_reg = 0x06; /* CTRL3 register*/
    Write_Register((uint8_t) REG_CTRL3, (uint16_t) getDriverReg.ctrl3_reg);

    getDriverReg.ctrl2_reg = 0x30; /* CTRL4 register*/
    Write_Register((uint8_t) REG_CTRL4, (uint16_t) getDriverReg.ctrl4_reg);

    getDriverReg.ctrl1_reg = 0x08; /* CTRL5 register*/
    Write_Register((uint8_t) REG_CTRL5, (uint16_t) getDriverReg.ctrl5_reg);

    getDriverReg.ctrl2_reg = 0x03; /* CTRL6 register*/
    Write_Register((uint8_t) REG_CTRL6, (uint16_t) getDriverReg.ctrl6_reg);

    getDriverReg.ctrl1_reg = 0x20; /* CTRL7 register*/
    Write_Register((uint8_t) REG_CTRL7, (uint16_t) getDriverReg.ctrl7_reg);

    /* CTRL8 register >> CTRL8 = 0b11111111;  //0xFF [read only] */
    /* CTRL9 register >> CTRL9 = 0b00001111;  //0x0F [read only]*/
}

void clearFaults(void) {

    getDriverReg.ctrl4_reg |= CLR_FLT_MASK; //Write '1' to this bit to clear all latched fault bits. 
    Write_Register((uint8_t) REG_CTRL4, (uint16_t) getDriverReg.ctrl4_reg);

}

void enableDriver() {
    getDriverReg.ctrl2_reg |= EN_OUT_MASK;
    Write_Register(REG_CTRL2, getDriverReg.ctrl2_reg);
}

void disableDriver() {
    getDriverReg.ctrl2_reg &= ~(EN_OUT_MASK);
    Write_Register(REG_CTRL2, getDriverReg.ctrl2_reg);

}
// ### A simple SPI interface with STEP/DIR pins allows an external controller to manage the direction and step rate of the stepper motor.###


// Enables direction control through SPI (SPI_DIR = 1), allowing setDirection() to override the DIR pin.

void enableSPIDirection() {
    getDriverReg.ctrl3_reg |= SPI_DIR_MASK;
    Write_Register(REG_CTRL3, getDriverReg.ctrl3_reg);

}

// Disables direction control through SPI (SPI_DIR = 0), making the DIR pin control direction instead.

void disableSPIDirection() {
    getDriverReg.ctrl3_reg &= ~(SPI_DIR_MASK);
    Write_Register(REG_CTRL3, getDriverReg.ctrl3_reg);
}

// Enables stepping through SPI (SPI_STEP = 1), allowing step() to override the STEP pin.

void enableSPIStep() {
    getDriverReg.ctrl3_reg |= SPI_STEP_MASK;
    Write_Register(REG_CTRL3, getDriverReg.ctrl3_reg);
}

// Disables stepping through SPI (SPI_STEP = 0), making the STEP pin control stepping instead.

void disableSPIStep() {
    getDriverReg.ctrl3_reg |= SPI_STEP_MASK;
    Write_Register(REG_CTRL3, getDriverReg.ctrl3_reg);
}

// Sets the motor direction (DIR) >>>>> Allowed values are 0 or 1.
// You must first call enableSPIDirection() to allow the direction to be controlled through SPI.  
//enableSPIDirection() >> can use this command to control the direction of the stepper motor and leave the DIR pin disconnected.

void setDirection(bool value) {
    if (value) {
        getDriverReg.ctrl3_reg |= DIR_MASK;
        Write_Register(REG_CTRL3, getDriverReg.ctrl3_reg);

    } else {
        getDriverReg.ctrl3_reg &= ~(DIR_MASK);
        Write_Register(REG_CTRL3, getDriverReg.ctrl3_reg);
    }
}

//The MICROSTEP_MODE bits in the SPI register are used to configure the step mode

void setSteppingMode(uint8_t mode) {
    switch (mode) {
        case 0: getDriverReg.ctrl3_reg |= MicroStep1_100 & 0x0F;
            break;
        case 1: getDriverReg.ctrl3_reg |= MicroStep1 & 0x0F;
            break;
        case 2: getDriverReg.ctrl3_reg |= MicroStep2 & 0x0F;
            break;
        case 3: getDriverReg.ctrl3_reg |= MicroStep2_NC & 0x0F;
            break;
        case 4: getDriverReg.ctrl3_reg |= MicroStep4 & 0x0F;
            break;
        case 8: getDriverReg.ctrl3_reg |= MicroStep8 & 0x0F;
            break;
        case 16: getDriverReg.ctrl3_reg |= MicroStep16 & 0x0F;
            break;
        case 32: getDriverReg.ctrl3_reg |= MicroStep32 & 0x0F;
            break;
        case 64: getDriverReg.ctrl3_reg |= MicroStep64 & 0x0F;
            break;
        case 128: getDriverReg.ctrl3_reg |= MicroStep128 & 0x0F;
            break;
        case 256: getDriverReg.ctrl3_reg |= MicroStep256 & 0x0F;
            break;
        default: getDriverReg.ctrl3_reg |= MicroStep16 & 0x0F; // Invalid mode; pick 1/16 micro-step by default.  
            break;
    }
    Write_Register(REG_CTRL3, getDriverReg.ctrl3_reg);
}


// Configure the starting speed

void setupSpeed(void) {

}

/* 
 * Conditions to Enable Output Drivers
 *  nSLEEP = 1 
 *  ENABLE = 1 
 *  EN_OUT = 1
 * *****************************************************************************************
 Device Functional Modes
 [#] Sleep Mode (nSLEEP = 0)
 [#] Disable Mode (nSLEEP = 1, ENABLE = 0)
 * >>>>>>>>> The ENABLE pin is used to enable or disable the half bridges in the device.
 * >>>>>>>>> When the ENABLE pin is low, the output drivers are disabled in the Hi-Z state.
 * >>>>>>>>> The EN_OUT bit can also be used to disable the output drivers.
 *           When the EN_OUT bit is '0', the output drivers are disabled in the Hi-Z state. 
 * +++++ see Table 7-11. Conditions to Enable or Disable Output Drivers on DataSheet.
 * ****************************************************************************************** 
 [#] Operating Mode (nSLEEP = 1, ENABLE = 1)
 * >>>>>>>>> the device enters the active mode When the nSLEEP pin is high, the ENABLE pin is 1
 * >>>>>>>>> The tWAKE time must elapse before the device is ready for inputs.
 * >>>>>>>>> tWAKE = 1.2 ms
 * ******************************************************************************************
 * nFAULT pin will be high after power-up.
 * The nFAULT pin will be logic low when a fault is detected.
 
 
 */

void OperatingMode() {
    // nFAULT pin will be high after power-up.
    //The nFAULT pin will be logic low when a fault is detected.
    if (nFAULT_GetValue == 0) // Fault Condition
    {
        nFaultFlag = 0;
        // read the FAULT_STATUS Register
        getDriverReg.fault_status_reg = Read_Register((uint8_t) REG_FAULT_STATUS);
    } else
        nFaultFlag = 1;

    // the device enters the active mode When the nSLEEP pin is high, the ENABLE pin is 1
    SLEEP_high();

    //The tWAKE time must elapse before the device is ready for inputs.(set enable high)
    //tWAKE is Wake-up time for  nSLEEP = 1 set  1.2 ms
    __delay_ms(1.2);

    // set ENABLE pin high to enable device
    ENABLE_high();

    //// bit 7  EN_OUT check its value 
    if ((getDriverReg.ctrl2_reg & EN_OUT_MASK) == 0) //'0' disable all outputs.
    {
        enableDriver();
    }

}
/*
 * CONFIGURATION >>>  nSLEEP pin = 0
 *   // disable SPI actions when asleep or outputs disabled
 
 */
void SleepMode()
{


}


// Motor Functions