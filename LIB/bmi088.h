#ifndef __BMI088_H
#define __BMI088_H

#include "spi.h"
#include "delay.h"
#include "sys.h"
#include "math.h"

#define ACC_CS_LOW 	PAout(4)=0
#define ACC_CS_HIGH PAout(4)=1
#define GYRO_CS_LOW PCout(4)=0
#define GYRO_CS_HIGH PCout(4)=1
#define ACC_RANGE BMI088_ACCEL_RANGE_24G 
#define GYRO_RANGE BMI08X_GYRO_BW_230_ODR_2000_HZ

#define ACC 1
#define GYRO 0

//陀螺仪数据结构体
typedef struct
{
	int16_t gyro[3];	//0x02-0x07
}__attribute__((__packed__))bmi088_gyroData_t;
typedef bmi088_gyroData_t* bmi088_gyroData_p;
//加速度数据结构体
typedef struct bmi088_accelData_t 
{
	uint8_t rsv;
	int16_t acc[3];	//0x12-0x17
	uint32_t sensor_time:24;	//0x18-0x1a
	uint8_t rsv2[2];	//0x1b-0x1c
	uint8_t acc_int_stat;	//0x1d
	uint8_t rsv3[4];	//0x1e-0x21
	uint16_t temperature;	//0x22-0x23
}__attribute__((__packed__))bmi088_accelData_t;
typedef bmi088_accelData_t* bmi088_accelData_p;
typedef struct 
{
	double accX,accY,accZ;
	float temperature;
}__attribute__((__packed__))accData_t;
typedef accData_t* accData_p;

typedef struct
{
	double gyroX_rad,gyroY_rad,gyroZ_rad;
	double gyroX_deg,gyroY_deg,gyroZ_deg;
}__attribute__((__packed__))gyroData_t;
typedef gyroData_t* gyroData_p;





int8_t bmi088_init(void);
uint8_t bmi088_write_byte(uint8_t reg,uint8_t val,uint8_t sensor);
uint8_t bmi088_read_byte(uint8_t reg,uint8_t sensor);
void bmi088_read(uint8_t reg,uint8_t* buffer,uint8_t len,uint8_t sensor);
void bmi088_dataRead(bmi088_accelData_p bmi088_accData,bmi088_gyroData_p bmi088_gyroData);
void bmi088_dataTransform(bmi088_accelData_p bmi088_accData,bmi088_gyroData_p 
	bmi088_gyroData,accData_p accData,gyroData_p gyroData);





/*************************** BMI08 Accelerometer Macros *****************************/

/** Register map */
/* Accel registers */

/**\name    Accel Chip Id register */
#define BMI08X_ACCEL_CHIP_ID_REG                    UINT8_C(0x00)

/**\name    Accel Error condition register */
#define BMI08X_ACCEL_ERR_REG                        UINT8_C(0x02)

/**\name    Accel Status flag register */
#define BMI08X_ACCEL_STATUS_REG                     UINT8_C(0x03)

/**\name    Accel X LSB data register */
#define BMI08X_ACCEL_X_LSB_REG                      UINT8_C(0x12)

/**\name    Accel X MSB data register */
#define BMI08X_ACCEL_X_MSB_REG                      UINT8_C(0x13)

/**\name    Accel Y LSB data register */
#define BMI08X_ACCEL_Y_LSB_REG                      UINT8_C(0x14)

/**\name    Accel Y MSB data register */
#define BMI08X_ACCEL_Y_MSB_REG                      UINT8_C(0x15)

/**\name    Accel Z LSB data register */
#define BMI08X_ACCEL_Z_LSB_REG                      UINT8_C(0x16)

/**\name    Accel Z MSB data register */
#define BMI08X_ACCEL_Z_MSB_REG                      UINT8_C(0x17)

/**\name    Sensor time byte 0 register */
#define BMI08X_ACCEL_SENSORTIME_0_REG               UINT8_C(0x18)

/**\name    Sensor time byte 1 register */
#define BMI08X_ACCEL_SENSORTIME_1_REG               UINT8_C(0x19)

/**\name    Sensor time byte 2 register */
#define BMI08X_ACCEL_SENSORTIME_2_REG               UINT8_C(0x1A)

/**\name    Accel Interrupt status0 register */
#define BMI08X_ACCEL_INT_STAT_0_REG                 UINT8_C(0x1C)

/**\name    Accel Interrupt status1 register */
#define BMI08X_ACCEL_INT_STAT_1_REG                 UINT8_C(0x1D)

/**\name    Accel general purpose register 0*/
#define BMI08X_ACCEL_GP_0_REG                       UINT8_C(0x1E)

/**\name    Sensor temperature MSB data register */
#define BMI08X_TEMP_MSB_REG                         UINT8_C(0x22)

/**\name    Sensor temperature LSB data register */
#define BMI08X_TEMP_LSB_REG                         UINT8_C(0x23)

/**\name    Accel general purpose register 4*/
#define BMI08X_ACCEL_GP_4_REG                       UINT8_C(0x27)

/**\name    Accel Internal status register */
#define BMI08X_ACCEL_INTERNAL_STAT_REG              UINT8_C(0x2A)

/**\name    Accel configuration register */
#define BMI08X_ACCEL_CONF_REG                       UINT8_C(0x40)

/**\name    Accel range setting register */
#define BMI08X_ACCEL_RANGE_REG                      UINT8_C(0x41)

/**\name    Accel Interrupt pin 1 configuration register */
#define BMI08X_ACCEL_INT1_IO_CONF_REG               UINT8_C(0x53)

/**\name    Accel Interrupt pin 2 configuration register */
#define BMI08X_ACCEL_INT2_IO_CONF_REG               UINT8_C(0x54)

/**\name    Accel Interrupt latch configuration register */
#define BMI08X_ACCEL_INT_LATCH_CONF_REG             UINT8_C(0x55)

/**\name    Accel Interrupt pin1 mapping register */
#define BMI08X_ACCEL_INT1_MAP_REG                   UINT8_C(0x56)

/**\name    Accel Interrupt pin2 mapping register */
#define BMI08X_ACCEL_INT2_MAP_REG                   UINT8_C(0x57)

/**\name    Accel Interrupt map register */
#define BMI08X_ACCEL_INT1_INT2_MAP_DATA_REG         UINT8_C(0x58)

/**\name    Accel Init control register */
#define BMI08X_ACCEL_INIT_CTRL_REG                  UINT8_C(0x59)

/**\name    Accel Self test register */
#define BMI08X_ACCEL_SELF_TEST_REG                  UINT8_C(0x6D)

/**\name    Accel Power mode configuration register */
#define BMI08X_ACCEL_PWR_CONF_REG                   UINT8_C(0x7C)

/**\name    Accel Power control (switch on or off ) register */
#define BMI08X_ACCEL_PWR_CTRL_REG                   UINT8_C(0x7D)

/**\name    Accel Soft reset register */
#define BMI08X_ACCEL_SOFTRESET_REG                  UINT8_C(0x7E)

#if BMI08X_FEATURE_BMI085 == 1
/**\name    BMI085 Accel unique chip identifier */
#define BMI08X_ACCEL_CHIP_ID                        UINT8_C(0x1F)
#elif BMI08X_FEATURE_BMI088 == 1
/**\name    BMI088 Accel unique chip identifier */
#endif
#define BMI08X_ACCEL_CHIP_ID                        UINT8_C(0x1E)


/**\name    Accel I2C slave address */
#define BMI08X_ACCEL_I2C_ADDR_PRIMARY               UINT8_C(0x18)
#define BMI08X_ACCEL_I2C_ADDR_SECONDARY             UINT8_C(0x19)

/**\name    Feature Config related Registers */
#define BMI08X_ACCEL_RESERVED_5B_REG                UINT8_C(0x5B)
#define BMI08X_ACCEL_RESERVED_5C_REG                UINT8_C(0x5C)
#define BMI08X_ACCEL_FEATURE_CFG_REG                UINT8_C(0x5E)

/**\name    Interrupt masks */
#define BMI08X_ACCEL_DATA_READY_INT                 UINT8_C(0x80)

/**\name    Accel Bandwidth */
#define BMI08X_ACCEL_BW_OSR4                        UINT8_C(0x00)
#define BMI08X_ACCEL_BW_OSR2                        UINT8_C(0x01)
#define BMI08X_ACCEL_BW_NORMAL                      UINT8_C(0x02)

/**\name    BMI085 Accel Range */
#define BMI085_ACCEL_RANGE_2G                       UINT8_C(0x00)
#define BMI085_ACCEL_RANGE_4G                       UINT8_C(0x01)
#define BMI085_ACCEL_RANGE_8G                       UINT8_C(0x02)
#define BMI085_ACCEL_RANGE_16G                      UINT8_C(0x03)


/**\name  BMI088 Accel Range */
#define BMI088_ACCEL_RANGE_3G                       UINT8_C(0x00)
#define BMI088_ACCEL_RANGE_6G                       UINT8_C(0x01)
#define BMI088_ACCEL_RANGE_12G                      UINT8_C(0x02)
#define BMI088_ACCEL_RANGE_24G                      UINT8_C(0x03)

/**\name    Accel Output data rate */
#define BMI08X_ACCEL_ODR_12_5_HZ                    UINT8_C(0x05)
#define BMI08X_ACCEL_ODR_25_HZ                      UINT8_C(0x06)
#define BMI08X_ACCEL_ODR_50_HZ                      UINT8_C(0x07)
#define BMI08X_ACCEL_ODR_100_HZ                     UINT8_C(0x08)
#define BMI08X_ACCEL_ODR_200_HZ                     UINT8_C(0x09)
#define BMI08X_ACCEL_ODR_400_HZ                     UINT8_C(0x0A)
#define BMI08X_ACCEL_ODR_800_HZ                     UINT8_C(0x0B)
#define BMI08X_ACCEL_ODR_1600_HZ                    UINT8_C(0x0C)

/**\name    Accel Self test */
#define BMI08X_ACCEL_SWITCH_OFF_SELF_TEST           UINT8_C(0x00)
#define BMI08X_ACCEL_POSITIVE_SELF_TEST             UINT8_C(0x0D)
#define BMI08X_ACCEL_NEGATIVE_SELF_TEST             UINT8_C(0x09)

/**\name    Accel Power mode */
#define BMI08X_ACCEL_PM_ACTIVE                      UINT8_C(0x00)
#define BMI08X_ACCEL_PM_SUSPEND                     UINT8_C(0x03)

/**\name    Accel Power control settings */
#define BMI08X_ACCEL_POWER_DISABLE                  UINT8_C(0x00)
#define BMI08X_ACCEL_POWER_ENABLE                   UINT8_C(0x04)

/**\name    Accel internal interrupt pin mapping */
#define BMI08X_ACCEL_INTA_DISABLE                   UINT8_C(0x00)
#define BMI08X_ACCEL_INTA_ENABLE                    UINT8_C(0x01)
#define BMI08X_ACCEL_INTB_DISABLE                   UINT8_C(0x00)
#define BMI08X_ACCEL_INTB_ENABLE                    UINT8_C(0x02)

/**\name    Accel Soft reset delay */
#define BMI08X_ACCEL_SOFTRESET_DELAY_MS             UINT8_C(1)

/**\name    Mask definitions for ACCEL_ERR_REG register */
#define BMI08X_FATAL_ERR_MASK                       UINT8_C(0x01)
#define BMI08X_ERR_CODE_MASK                        UINT8_C(0x1C)

/**\name    Position definitions for ACCEL_ERR_REG register */
#define BMI08X_CMD_ERR_POS                          UINT8_C(1)
#define BMI08X_ERR_CODE_POS                         UINT8_C(2)

/**\name    Mask definition for ACCEL_STATUS_REG register */
#define BMI08X_ACCEL_STATUS_MASK                    UINT8_C(0x80)

/**\name    Position definitions for ACCEL_STATUS_REG  */
#define BMI08X_ACCEL_STATUS_POS                     UINT8_C(7)

/**\name    Mask definitions for odr, bandwidth and range */
#define BMI08X_ACCEL_ODR_MASK                       UINT8_C(0x0F)
#define BMI08X_ACCEL_BW_MASK                        UINT8_C(0x70)
#define BMI08X_ACCEL_RANGE_MASK                     UINT8_C(0x03)

/**\name    Position definitions for odr, bandwidth and range */
#define BMI08X_ACCEL_BW_POS                         UINT8_C(4)

/**\name    Mask definitions for INT1_IO_CONF register */
#define BMI08X_ACCEL_INT_EDGE_MASK                  UINT8_C(0x01)
#define BMI08X_ACCEL_INT_LVL_MASK                   UINT8_C(0x02)
#define BMI08X_ACCEL_INT_OD_MASK                    UINT8_C(0x04)
#define BMI08X_ACCEL_INT_IO_MASK                    UINT8_C(0x08)
#define BMI08X_ACCEL_INT_IN_MASK                    UINT8_C(0x10)

/**\name    Position definitions for INT1_IO_CONF register */
#define BMI08X_ACCEL_INT_EDGE_POS                   UINT8_C(0)
#define BMI08X_ACCEL_INT_LVL_POS                    UINT8_C(1)
#define BMI08X_ACCEL_INT_OD_POS                     UINT8_C(2)
#define BMI08X_ACCEL_INT_IO_POS                     UINT8_C(3)
#define BMI08X_ACCEL_INT_IN_POS                     UINT8_C(4)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define BMI08X_ACCEL_MAP_INTA_MASK                  UINT8_C(0x01)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define BMI08X_ACCEL_MAP_INTA_POS                   UINT8_C(0x00)

/**\name    Mask definitions for INT1_INT2_MAP_DATA register */
#define BMI08X_ACCEL_INT1_DRDY_MASK                 UINT8_C(0x04)
#define BMI08X_ACCEL_INT2_DRDY_MASK                 UINT8_C(0x40)

/**\name    Position definitions for INT1_INT2_MAP_DATA register */
#define BMI08X_ACCEL_INT1_DRDY_POS                  UINT8_C(2)
#define BMI08X_ACCEL_INT2_DRDY_POS                  UINT8_C(6)

/**\name    Asic Initialization value */
#define BMI08X_ASIC_INITIALIZED                     UINT8_C(0x01)

/*************************** BMI08 Gyroscope Macros *****************************/
/** Register map */
/* Gyro registers */

/**\name    Gyro Chip Id register */
#define BMI08X_GYRO_CHIP_ID_REG                     UINT8_C(0x00)

/**\name    Gyro X LSB data register */
#define BMI08X_GYRO_X_LSB_REG                       UINT8_C(0x02)

/**\name    Gyro X MSB data register */
#define BMI08X_GYRO_X_MSB_REG                       UINT8_C(0x03)

/**\name    Gyro Y LSB data register */
#define BMI08X_GYRO_Y_LSB_REG                       UINT8_C(0x04)

/**\name    Gyro Y MSB data register */
#define BMI08X_GYRO_Y_MSB_REG                       UINT8_C(0x05)

/**\name    Gyro Z LSB data register */
#define BMI08X_GYRO_Z_LSB_REG                       UINT8_C(0x06)

/**\name    Gyro Z MSB data register */
#define BMI08X_GYRO_Z_MSB_REG                       UINT8_C(0x07)

/**\name    Gyro Interrupt status register */
#define BMI08X_GYRO_INT_STAT_1_REG                  UINT8_C(0x0A)

/**\name    Gyro Range register */
#define BMI08X_GYRO_RANGE_REG                       UINT8_C(0x0F)

/**\name    Gyro Bandwidth register */
#define BMI08X_GYRO_BANDWIDTH_REG                   UINT8_C(0x10)

/**\name    Gyro Power register */
#define BMI08X_GYRO_LPM1_REG                        UINT8_C(0x11)

/**\name    Gyro Soft reset register */
#define BMI08X_GYRO_SOFTRESET_REG                   UINT8_C(0x14)

/**\name    Gyro Interrupt control register */
#define BMI08X_GYRO_INT_CTRL_REG                    UINT8_C(0x15)

/**\name    Gyro Interrupt Pin configuration register */
#define BMI08X_GYRO_INT3_INT4_IO_CONF_REG           UINT8_C(0x16)

/**\name    Gyro Interrupt Map register */
#define BMI08X_GYRO_INT3_INT4_IO_MAP_REG            UINT8_C(0x18)

/**\name    Gyro Self test register */
#define BMI08X_GYRO_SELF_TEST_REG                   UINT8_C(0x3C)

/**\name    Gyro unique chip identifier */
#define BMI08X_GYRO_CHIP_ID                         UINT8_C(0x0F)

/**\name    Gyro I2C slave address */
#define BMI08X_GYRO_I2C_ADDR_PRIMARY                UINT8_C(0x68)
#define BMI08X_GYRO_I2C_ADDR_SECONDARY              UINT8_C(0x69)

/**\name    Gyro Range */
#define BMI08X_GYRO_RANGE_2000_DPS                  UINT8_C(0x00)
#define BMI08X_GYRO_RANGE_1000_DPS                  UINT8_C(0x01)
#define BMI08X_GYRO_RANGE_500_DPS                   UINT8_C(0x02)
#define BMI08X_GYRO_RANGE_250_DPS                   UINT8_C(0x03)
#define BMI08X_GYRO_RANGE_125_DPS                   UINT8_C(0x04)

/**\name    Gyro Output data rate and bandwidth */
#define BMI08X_GYRO_BW_532_ODR_2000_HZ              UINT8_C(0x00)
#define BMI08X_GYRO_BW_230_ODR_2000_HZ              UINT8_C(0x01)
#define BMI08X_GYRO_BW_116_ODR_1000_HZ              UINT8_C(0x02)
#define BMI08X_GYRO_BW_47_ODR_400_HZ                UINT8_C(0x03)
#define BMI08X_GYRO_BW_23_ODR_200_HZ                UINT8_C(0x04)
#define BMI08X_GYRO_BW_12_ODR_100_HZ                UINT8_C(0x05)
#define BMI08X_GYRO_BW_64_ODR_200_HZ                UINT8_C(0x06)
#define BMI08X_GYRO_BW_32_ODR_100_HZ                UINT8_C(0x07)
#define BMI08X_GYRO_ODR_RESET_VAL                   UINT8_C(0x80)

/**\name    Gyro Power mode */
#define BMI08X_GYRO_PM_NORMAL                       UINT8_C(0x00)
#define BMI08X_GYRO_PM_DEEP_SUSPEND                 UINT8_C(0x20)
#define BMI08X_GYRO_PM_SUSPEND                      UINT8_C(0x80)

/**\name    Gyro data ready interrupt enable value */
#define BMI08X_GYRO_DRDY_INT_DISABLE_VAL            UINT8_C(0x00)
#define BMI08X_GYRO_DRDY_INT_ENABLE_VAL             UINT8_C(0x80)

/**\name    Gyro data ready map values */
#define BMI08X_GYRO_MAP_DRDY_TO_INT3                UINT8_C(0x01)
#define BMI08X_GYRO_MAP_DRDY_TO_INT4                UINT8_C(0x80)
#define BMI08X_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4      UINT8_C(0x81)

/**\name    Gyro Soft reset delay */
#define BMI08X_GYRO_SOFTRESET_DELAY                 UINT8_C(30)
/**\name    Gyro power mode config delay */
#define BMI08X_GYRO_POWER_MODE_CONFIG_DELAY         UINT8_C(30)

/** Mask definitions for range, bandwidth and power */
#define BMI08X_GYRO_RANGE_MASK                      UINT8_C(0x07)
#define BMI08X_GYRO_BW_MASK                         UINT8_C(0x0F)
#define BMI08X_GYRO_POWER_MASK                      UINT8_C(0xA0)

/** Position definitions for range, bandwidth and power */
#define BMI08X_GYRO_POWER_POS                       UINT8_C(5)

/**\name    Mask definitions for BMI08X_GYRO_INT_CTRL_REG register */
#define BMI08X_GYRO_DATA_EN_MASK                    UINT8_C(0x80)

/**\name    Position definitions for BMI08X_GYRO_INT_CTRL_REG register */
#define BMI08X_GYRO_DATA_EN_POS                     UINT8_C(7)

/**\name    Mask definitions for BMI08X_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI08X_GYRO_INT3_LVL_MASK                   UINT8_C(0x01)
#define BMI08X_GYRO_INT3_OD_MASK                    UINT8_C(0x02)
#define BMI08X_GYRO_INT4_LVL_MASK                   UINT8_C(0x04)
#define BMI08X_GYRO_INT4_OD_MASK                    UINT8_C(0x08)

/**\name    Position definitions for BMI08X_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI08X_GYRO_INT3_OD_POS                     UINT8_C(1)
#define BMI08X_GYRO_INT4_LVL_POS                    UINT8_C(2)
#define BMI08X_GYRO_INT4_OD_POS                     UINT8_C(3)

/**\name    Mask definitions for BMI08X_GYRO_INT_EN_REG register */
#define BMI08X_GYRO_INT_EN_MASK                     UINT8_C(0x80)

/**\name    Position definitions for BMI08X_GYRO_INT_EN_REG register */
#define BMI08X_GYRO_INT_EN_POS                      UINT8_C(7)

/**\name    Mask definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI08X_GYRO_INT3_MAP_MASK                   UINT8_C(0x01)
#define BMI08X_GYRO_INT4_MAP_MASK                   UINT8_C(0x80)

/**\name    Position definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI08X_GYRO_INT3_MAP_POS                    UINT8_C(0)
#define BMI08X_GYRO_INT4_MAP_POS                    UINT8_C(7)

/**\name    Mask definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI088_GYRO_INT3_MAP_MASK                   UINT8_C(0x01)
#define BMI088_GYRO_INT4_MAP_MASK                   UINT8_C(0x80)

/**\name    Position definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI088_GYRO_INT3_MAP_POS                    UINT8_C(0)
#define BMI088_GYRO_INT4_MAP_POS                    UINT8_C(7)
/**\name    Mask definitions for GYRO_SELF_TEST register */
#define BMI08X_GYRO_SELF_TEST_EN_MASK               UINT8_C(0x01)
#define BMI08X_GYRO_SELF_TEST_RDY_MASK              UINT8_C(0x02)
#define BMI08X_GYRO_SELF_TEST_RESULT_MASK           UINT8_C(0x04)
#define BMI08X_GYRO_SELF_TEST_FUNCTION_MASK         UINT8_C(0x08)

/**\name    Position definitions for GYRO_SELF_TEST register */
#define BMI08X_GYRO_SELF_TEST_RDY_POS               UINT8_C(1)
#define BMI08X_GYRO_SELF_TEST_RESULT_POS            UINT8_C(2)
#define BMI08X_GYRO_SELF_TEST_FUNCTION_POS          UINT8_C(3)

#endif

