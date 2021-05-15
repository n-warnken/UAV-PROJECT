/*
 * BMI088.h
 *
 *  Created on: Jan 18, 2021
 *      Author: 61416
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include <stddef.h>
#include <_ansi.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#define BMI088_ACC_ADDRESS          0x19

#define BMI088_ACC_CHIP_ID          0x00 // Default value 0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_X_MSB            0x13
#define BMI088_ACC_Y_LSB            0x14
#define BMI088_ACC_Y_MSB            0x15
#define BMI088_ACC_Z_LSB            0x16
#define BMI088_ACC_Z_MSB            0x17

#define BMI088_ACC_SENSOR_TIME_0    0x18
#define BMI088_ACC_SENSOR_TIME_1    0x19
#define BMI088_ACC_SENSOR_TIME_2    0x1A

#define BMI088_ACC_INT_STAT_1       0x1D

#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41

#define BMI088_ACC_INT1_IO_CTRL     0x53
#define BMI088_ACC_INT2_IO_CTRL     0x54
#define BMI088_ACC_INT_MAP_DATA     0x58

#define BMI088_ACC_SELF_TEST        0x6D

#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRl         0x7D

#define BMI088_ACC_SOFT_RESET       0x7E

#define BMI088_GYRO_ADDRESS             0x69

#define BMI088_GYRO_CHIP_ID             0x00 // Default value 0x0F

#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07

#define BMI088_GYRO_INT_STAT_1          0x0A

#define BMI088_GYRO_RANGE               0x0F
#define BMI088_GYRO_BAND_WIDTH          0x10

#define BMI088_GYRO_LPM_1               0x11

#define BMI088_GYRO_SOFT_RESET          0x14

#define BMI088_GYRO_INT_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18

#define BMI088_GYRO_SELF_TEST           0x3C


#define BMI088_SPI_PORT 	   			hspi3

#define BMI088_CSB1_A_PORT        		GPIOC
#define BMI088_CSB1_A_PIN         		GPIO_PIN_12

#define BMI088_CSB2_G_PORT         		GPIOD
#define BMI088_CSB2_G_PIN          		GPIO_PIN_2

extern SPI_HandleTypeDef BMI088_SPI_PORT;

typedef enum { // device type
    ACC = 0x00, //
    GYRO = 0x01, //
} device_type_t;

typedef enum { // measurement rage
    RANGE_3G = 0x00, //
    RANGE_6G = 0x01, //
    RANGE_12G = 0x02, //
    RANGE_24G = 0x03, //
} acc_scale_type_t;

typedef enum { // output data rate
    ODR_12 = 0x05, //
    ODR_25 = 0x06, //
    ODR_50 = 0x07, //
    ODR_100 = 0x08, //
    ODR_200 = 0x09, //
    ODR_400 = 0x0A, //
    ODR_800 = 0x0B, //
    ODR_1600 = 0x0C, //
} acc_odr_type_t;

typedef enum { // power mode
    ACC_ACTIVE = 0x00, //
    ACC_SUSPEND = 0x03, //
} acc_power_type_t;

typedef enum { // measurement rage
    RANGE_2000 = 0x00, //
    RANGE_1000 = 0x01, //
    RANGE_500 = 0x02, //
    RANGE_250 = 0x03, //
    RANGE_125 = 0x04, //
} gyro_scale_type_t;

typedef enum { // output data rate
    ODR_2000_BW_532 = 0x00, //
    ODR_2000_BW_230 = 0x01, //
    ODR_1000_BW_116 = 0x02, //
    ODR_400_BW_47 = 0x03, //
    ODR_200_BW_23 = 0x04, //
    ODR_100_BW_12 = 0x05, //
    ODR_200_BW_64 = 0x06, //
    ODR_100_BW_32 = 0x07, //
} gyro_odr_type_t;

typedef enum { // power mode
    GYRO_NORMAL = 0x00, //
    GYRO_SUSPEND = 0x80, //
    GYRO_DEEP_SUSPEND = 0x20, //
} gyro_power_type_t;


extern int isConnection(void);

extern void BMI088_Init(void);

int BMI088_isConnection(void);

extern void BMI088_setAccPoweMode(acc_power_type_t mode);
extern void BMI088_setGyroPoweMode(gyro_power_type_t mode);

extern void BMI088_setAccScaleRange(acc_scale_type_t range);
extern void BMI088_setAccOutputDataRate(acc_odr_type_t odr);

extern void BMI088_setGyroScaleRange(gyro_scale_type_t range);
extern void BMI088_setGyroOutputDataRate(gyro_odr_type_t odr);

extern void BMI088_getAcceleration(float* x, float* y, float* z);
extern float BMI088_getAccelerationX(void);
extern float BMI088_getAccelerationY(void);
extern float BMI088_getAccelerationZ(void);

extern void BMI088_getGyroscope(float* x, float* y, float* z);
extern float BMI088_getGyroscopeX(void);
extern float BMI088_getGyroscopeY(void);
extern float BMI088_getGyroscopeZ(void);

extern int16_t BMI088_getTemperature(void);

extern uint8_t BMI088_getAccID(void);
extern uint8_t BMI088_getGyroID(void);

extern void BMI088_resetAcc(void);
extern void BMI088_resetGyro(void);


void BMI088_write8(device_type_t dev, uint8_t reg, uint8_t val);
uint8_t BMI088_read8(device_type_t dev, uint8_t reg);
uint16_t BMI088_read16(device_type_t dev, uint8_t reg);
uint32_t BMI088_read24(device_type_t dev, uint8_t reg);
void BMI088_read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len);

float accRange;
float gyroRange;
uint8_t devAddrAcc;
uint8_t devAddrGyro;


#endif /* INC_BMI088_H_ */
