
#ifndef _DRV_LIB_H
#define	_DRV_LIB_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>

/*
 ############## initializing the DRV8434S Register ##############
 */
typedef struct {
    /* Registers mapped to DRV8434S SPI registers*/
    uint8_t fault_status_reg; // Fault Status Register
    uint8_t diag_status1_reg; // Diagnostic register 1 for OCP
    uint8_t diag_status2_reg; // Diagnostic register 2 for other faults and warnings
    uint8_t ctrl1_reg; // CTRL1 register - Torque DAC, synchronous rectification, slew rate
    uint8_t ctrl2_reg; // CTRL2 register - Disable, TOFF, DECAY
    uint8_t ctrl3_reg; // CTRL3 register - Step, dir, microstep
    uint8_t ctrl4_reg; // CTRL4 register - Clear fault, lock, OL, OCP, OTSD, OTW
    uint8_t ctrl5_reg; // CTRL5 register - Stall detection configuration
    uint8_t ctrl6_reg; // CTRL6 register - Stall threshold[7:0]
    uint8_t ctrl7_reg; // CTRL7 register - RC ripple, Stall threshold[11:8]
    uint8_t ctrl8_reg; // CTRL8 register - Torque count[7:0]
    uint8_t ctrl9_reg; // CTRL9 register - Torque count[11:8]

} DRV8434S_REG_t;



#define WRITE_OPERATION				(0x0000)  // write command (W0 = 0),
#define READ_OPERATION				(0x4000)  // read command (W0 = 1),

/*SLEEP Mode*/
#define DRV8434S_SLEEP_MODE()             { LATFbits.LATF4 = 0; }

//========================================== 
// DRV8434S MICROSTEP MODE
#define  MicroStep1_100           0b0000 // Full step with 100% current
#define  MicroStep1               0b0001 // Full step with 71% current
#define  MicroStep2_NC            0b0010 // Non-circular 1/2 step
#define  MicroStep2               0b0011 // Circular 1/2 step
#define  MicroStep4               0b0100 // Circular 1/4 step
#define  MicroStep8               0b0101 // Circular 1/8 step
#define  MicroStep16              0b0110 // Circular 1/16 step
#define  MicroStep32              0b0111 // Circular 1/32 step
#define  MicroStep64              0b1000 // Circular 1/64 step
#define  MicroStep128             0b1001 // Circular 1/128 step
#define  MicroStep256             0b1010 // Circular 1/256 step
//=====================================================================


#define nSLEEP              TRISCbits.TRISC0         // Sleep pin
#define ENABLE             TRISFbits.TRISF5         // Driver output ENABLE pin
#define DIR                TRISAbits.TRISA4         // DIRection pin
#define STEP               TRISAbits.TRISA5         // STEP pin
#define nFAULT              TRISFbits.TRISF3         // Fault pin



#define CS                 LATFbits.LATF4
#define nSCS               RF4         // P3.0
#define SDI                RC4         // P3.1
#define SDO                RD1         // P3.2
#define SCLK               RC3         // P3.3


#define nFAULT_GetValue           PORTFbits.RF3

/* Enable SLEEP*/
#define SLEEP_high()             { LATFbits.LATF4 = 1; }
#define SLEEP_low()              { LATFbits.LATF4 = 0; }

/* Enable drive output*/
#define ENABLE_high()            { LATFbits.LATF4 = 1; }
#define ENABLE_low()             { LATFbits.LATF4 = 0; }

/* Enable DIR*/
#define DIR_high()               { LATFbits.LATF4 = 1; }
#define DIR_low()                { LATFbits.LATF4 = 0; }

/* Enable STEP*/
#define STEP_high()              { LATFbits.LATF4 = 1; }
#define STEP_low()               { LATFbits.LATF4 = 0; }


uint16_t Read_Register(uint8_t address);
uint16_t Write_Register(uint8_t address, uint16_t data);
void drv8434s_init(void);
void setDefaultRegisters(void);
void clearFaults(void);

void motor_set_microstep();
void setupSpeed(void);
void enableDriver();
void disableDriver();
void OperatingMode();
void SleepMode();
void setDirection(bool value);
void setSteppingMode(uint8_t mode);
void enableSPIDirection();
void enableSPIStep();

extern volatile DRV8434S_REG_t getDriverReg;
extern volatile uint8_t nFault_Pin;
extern volatile uint8_t nSleep_Pin;

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* _DRV_LIB_H */

