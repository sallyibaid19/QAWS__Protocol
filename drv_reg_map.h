
#ifndef DRV_REG_MAP_H
#define	DRV_REG_MAP_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

#define REG_FAULT_STATUS         (uint8_t)(0x00)         /* Fault Status Register */
#define REG_DIAG_STATUS_1        (uint8_t)(0x01)         /* Diagnostic register 1 for OCP   */
#define REG_DIAG_STATUS_2        (uint8_t)(0x02)         /* Diagnostic register 2 for other faults and warnings   */
#define REG_CTRL1                (uint8_t)(0x03)         /* CTRL1 register - Torque DAC, synchronous rectification, slew rate  */
#define REG_CTRL2                (uint8_t)(0x04)         /* CTRL2 register - Disable, TOFF, DECAY  */
#define REG_CTRL3                (uint8_t)(0x05)         /* CTRL3 register - Step, dir, microstep  */
#define REG_CTRL4                (uint8_t)(0x06)         /* CTRL4 register - Clear fault, lock, OL, OCP, OTSD, OTW  */
#define REG_CTRL5                (uint8_t)(0x07)         /* CTRL5 register - Stall detection configuration  */
#define REG_CTRL6                (uint8_t)(0x08)         /* CTRL6 register - Stall threshold */
#define REG_CTRL7                (uint8_t)(0x09)         /* CTRL7 register - Torque count */
#define REG_CTRL8                (uint8_t)(0x0A)         /* CTRL8 register - UTW, Rev ID */
#define REG_CTRL9                (uint8_t)(0x0B)         /* CTRL9 register - REV ID,TRQ COUNT*/

/* Mask for Register Bit Definitions */

/* SPI_REG_00 : IC fault Status Register */
#define OL_MASK               (0x01)         /*Indicates open load condition*/
#define OT_MASK               (0x02)         /*Logic OR of OTW, UTW, and OTSD.*/
#define STL_SLIP_MASK         (0x04)         /*Indicates motor stall or slip*/
#define OCP_MASK              (0x08)         /*Indicates undervoltage lockout fault condition*/
#define CPUV_MASK             (0x10)         /*Indicates charge pump undervoltage fault condition*/
#define UVLO_MASK             (0x20)         /*Indicates an undervoltage condition*/
#define SPI_ERROR_MASK        (0x40)         /*Indication SPI communication error*/
#define FAULT_MASK            (0x80)         /*Inverse of the nFAULT pin*/


/* SPI_REG_01 : Diagnostic register 1 for OCP */
#define OCP_HS1_A_MASK        (0x01)        /*Indicates overcurrent fault on the high-side FET of AOUT1 */
#define OCP_LS1_A_MASK        (0x02)        /*Indicates overcurrent fault on the low-side FET of AOUT1 */
#define OCP_HS2_A_MASK        (0x04)        /*Indicates overcurrent fault on the high-side FET of AOUT2 */
#define OCP_LS2_A_MASK        (0x08)        /*Indicates overcurrent fault on the low-side FET of AOUT2 */
#define OCP_HS1_B_MASK        (0x10)        /*Indicates overcurrent fault on the high-side FET of BOUT1 */
#define OCP_LS1_B_MASK        (0x20)        /*Indicates overcurrent fault on the low-side FET of BOUT1 */
#define OCP_HS2_B_MASK        (0x40)        /*Indicates overcurrent fault on the high-side FET of BOUT2 */
#define OCP_LS2_B_MASK        (0x80)        /*Indicates overcurrent fault on the low-side FET of BOUT2 */

/* SPI_REG_02 : Diagnostic register 2 for other faults and warnings */
#define OL_A_MASK             (0x01)        /*Indicates open-load detection on AOUT*/
#define OL_B_MASK             (0x02)        /*Indicates open-load detection on BOUT*/
#define SLIP_MASK             (0x04)        /*Indicates motor slip condition*/
#define STALL_MASK            (0x08)        /*Indicates motor stall condition*/
#define STL_LRN_OK_MASK       (0x10)        /*Indicates stall detection learning is successful*/
#define OTS_MASK              (0x20)        /*Indicates overtemperature shutdown*/
#define OTW_MASK              (0x40)        /*Indicates overtemperature warning */
#define UTW_MASK              (0x80)        /*Indicates undertemperature warning*/

/* SPI_REG_03 : CTRL1 register - Torque DAC, synchronous rectification, slew rate */
#define SLEW_RATE_MASK        (0x03)        /*Slew rate selection bits*/
#define EN_NSR_MASK           (0x04)        /*Enable synchronous rectification bits*/
#define RSVD1_MASK            (0x08)        /*TI reserved bit*/
#define TRQ_DAC_MASK          (0xF0)        /*Torque DAC bits*/

/* SPI_REG_04 : CTRL2 register - Disable, TOFF, DECAY */
#define DECAY_MASK            (0x07)        /*Bridge decay setting */
#define TOFF_MASK             (0x18)        /*Current regulation TOFF setting*/
#define RSVD2_MASK            (0x60)        /*TI reserved bits*/
#define EN_OUT_MASK          (0x80)         /*Hi-Z outputs bit. OR-ed with DRVOFF*/

/* SPI_REG_05 : CTRL3 register - Step, dir, microstep */
#define MICROSTEP_MODE_MASK   (0x0F)        /*Microstep setting*/
#define SPI_STEP_MASK         (0x10)        /*Enable SPI step control mode*/
#define SPI_DIR_MASK          (0x20)        /*Enable SPI direction control mode*/
#define STEP_MASK             (0x40)        /*Step control bit if SPI_STEP is enabled*/
#define DIR_MASK              (0x80)        /*Direction control bit if SPI_DIR is enabled*/

/* SPI_REG_06 : CTRL4 register - Clear fault, lock, OL, OCP, OTSD, OTW */
#define TW_REP_MASK           (0x01)        /*Report OTW and UTW on nFAULT*/
#define OTSD_MODE_MASK        (0x02)        /*OTSD latch fault setting*/
#define OCP_MODE_MASK         (0x04)        /*OCP latch fault setting*/
#define EN_OL_MASK            (0x08)        /*Open load detection enable*/
#define LOCK_MASK             (0x70)        /*Lock SPI registers*/
#define CLR_FLT_MASK          (0x80)        /*Clear all fault bits*/

/* SPI_REG_07 : CTRL5 register - Stall detection configuration */
#define STL_REP_MASK          (0x08)        /*Report stall detection on nFAULT*/
#define EN_STL_MASK           (0x10)        /*Enable stall detection*/
#define DIS_STL_MASK          (0xEF)        /*Disable stall detection - clears EN_STL bit*/
#define STL_LRN_MASK          (0x20)        /*Learn stall count threshold*/

/* SPI_REG_08 : CTRL6 register - Stall threshold (LSB) */
#define STALL_TH_MASK         (0xFF)        /*Stall Threshold[7:0] */

/* SPI_REG_09 : CTRL7 register - RC_RIPPLE, EN_SSC, stall threshold (MSB) */
#define STALL_TH_MASK_MSB     (0x0F)        /*Stall Threshold[11:8] */
#define EN_SSC                (0x20)        /*Spread-spectrum enable for CP and OSC*/
#define RC_RIPPLE             (0xC0)        /*Smart tune ripple control*/

/* SPI_REG_0A : CTRL8 register - Torque count */
#define TRQ_COUNT_MASK        (0xFF)        /*Torque Count[7:0]*/

/* SPI_REG_0B : CTRL9 register - UTW, Rev ID */
#define TRQ_COUNT_MASK_MSB    (0x0F)        /*Torque Count[11:8]*/
#define REV_ID_MASK           (0xF0)        /*Revision ID*/






#if(0)
#endif

/*******************************************************************************
 * Structure
 *******************************************************************************/




#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* DRV_REG_MAP_H */

