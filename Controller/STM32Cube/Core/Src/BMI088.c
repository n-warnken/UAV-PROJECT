/*
 * BMI088.c
 *
 *  Created on: Jan 18, 2021
 *      Author: 61416
 */


#include "BMI088.h"
#include <stdlib.h>

void BMI088_Init(void) {

	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_SET);
	BMI088_setAccScaleRange(RANGE_6G);
	BMI088_setAccOutputDataRate(ODR_100);
	BMI088_setAccPoweMode(ACC_ACTIVE);


	BMI088_setGyroScaleRange(RANGE_2000);
	BMI088_setGyroOutputDataRate(ODR_2000_BW_532);
	BMI088_setGyroPoweMode(GYRO_NORMAL);
}

int BMI088_isConnection(void) {
	uint8_t  test, test2;
	test = BMI088_getAccID();
	test2 = BMI088_getGyroID();
	return ((BMI088_getAccID() == 0x1E) && (BMI088_getGyroID() == 0x0F));
}

void BMI088_resetAcc(void) {
	BMI088_write8(ACC, BMI088_ACC_SOFT_RESET, 0xB6);
	BMI088_getAccID();
}

void BMI088_resetGyro(void) {
	BMI088_write8(GYRO, BMI088_GYRO_SOFT_RESET, 0xB6);
}

uint8_t BMI088_getAccID(void) {
    return BMI088_read8(ACC, BMI088_GYRO_CHIP_ID);
}

uint8_t BMI088_getGyroID(void) {
    return BMI088_read8(GYRO, BMI088_GYRO_CHIP_ID);
}

void BMI088_setAccPoweMode(acc_power_type_t mode) {
    if (mode == ACC_ACTIVE) {
    	BMI088_write8(ACC, BMI088_ACC_PWR_CTRl, 0x04);
    	BMI088_write8(ACC, BMI088_ACC_PWR_CONF, 0x00);
    } else if (mode == ACC_SUSPEND) {
    	BMI088_write8(ACC, BMI088_ACC_PWR_CONF, 0x03);
    	BMI088_write8(ACC, BMI088_ACC_PWR_CTRl, 0x00);
    }
}

void BMI088_setGyroPoweMode(gyro_power_type_t mode) {
    if (mode == GYRO_NORMAL) {
    	BMI088_write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
    } else if (mode == GYRO_SUSPEND) {
    	BMI088_write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
    } else if (mode == GYRO_DEEP_SUSPEND) {
    	BMI088_write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
    }
}

void BMI088_setAccScaleRange(acc_scale_type_t range) {
    if (range == RANGE_3G) {
        accRange = 3000;
    } else if (range == RANGE_6G) {
        accRange = 6000;
    } else if (range == RANGE_12G) {
        accRange = 12000;
    } else if (range == RANGE_24G) {
        accRange = 24000;
    }

    BMI088_write8(ACC, BMI088_ACC_RANGE, (uint8_t)range);
}

void BMI088_setAccOutputDataRate(acc_odr_type_t odr) {
    uint8_t data = 0;

    data = BMI088_read8(ACC, BMI088_ACC_CONF);
    data = data & 0xf0;
    data = data | (uint8_t)odr;

    BMI088_write8(ACC, BMI088_ACC_CONF, data);
}

void BMI088_setGyroScaleRange(gyro_scale_type_t range) {
    if (range == RANGE_2000) {
        gyroRange = 2000;
    } else if (range == RANGE_1000) {
        gyroRange = 1000;
    } else if (range == RANGE_500) {
        gyroRange = 500;
    } else if (range == RANGE_250) {
        gyroRange = 250;
    } else if (range == RANGE_125) {
        gyroRange = 125;
    }

    BMI088_write8(GYRO, BMI088_GYRO_RANGE, (uint8_t)range);
}

void BMI088_setGyroOutputDataRate(gyro_odr_type_t odr) {
	BMI088_write8(GYRO, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}

void BMI088_getAcceleration(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t ax = 0, ay = 0, az = 0;
    float value = 0;

    BMI088_read(ACC, BMI088_ACC_X_LSB, buf, 6);

    ax = buf[0] | (buf[1] << 8);
    ay = buf[2] | (buf[3] << 8);
    az = buf[4] | (buf[5] << 8);

    value = (int16_t)ax;
    *x = accRange * value / 32768;

    value = (int16_t)ay;
    *y = accRange * value / 32768;

    value = (int16_t)az;
    *z = accRange * value / 32768;
}

float BMI088_getAccelerationX(void) {
    uint16_t ax = 0;
    float value = 0;

    ax = BMI088_read16(ACC, BMI088_ACC_X_LSB);

    value = (int16_t)ax;
    value = accRange * value / 32768;

    return value;
}

float BMI088_getAccelerationY(void) {
    uint16_t ay = 0;
    float value = 0;

    ay = BMI088_read16(ACC, BMI088_ACC_Y_LSB);

    value = (int16_t)ay;
    value = accRange * value / 32768;

    return value;
}

float BMI088_getAccelerationZ(void) {
    uint16_t az = 0;
    float value = 0;

    az = BMI088_read16(ACC, BMI088_ACC_Z_LSB);

    value = (int16_t)az;
    value = accRange * value / 32768;

    return value;
}

void BMI088_getGyroscope(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t gx = 0, gy = 0, gz = 0;
    float value = 0;

    BMI088_read(GYRO, BMI088_GYRO_RATE_X_LSB, buf, 6);

    gx = buf[0] | (buf[1] << 8);
    gy = buf[2] | (buf[3] << 8);
    gz = buf[4] | (buf[5] << 8);

    value = (int16_t)gx;
    *x = gyroRange * value / 32768;

    value = (int16_t)gy;
    *y = gyroRange * value / 32768;

    value = (int16_t)gz;
    *z = gyroRange * value / 32768;
}

float BMI088_getGyroscopeX(void) {
    uint16_t gx = 0;
    float value = 0;

    gx = BMI088_read16(GYRO, BMI088_GYRO_RATE_X_LSB);

    value = (int16_t)gx;
    value = gyroRange * value / 32768;

    return value;
}

float BMI088_getGyroscopeY(void) {
    uint16_t gy = 0;
    float value = 0;

    gy = BMI088_read16(GYRO, BMI088_GYRO_RATE_Y_LSB);

    value = (int16_t)gy;
    value = gyroRange * value / 32768;

    return value;
}

float BMI088_getGyroscopeZ(void) {
    uint16_t gz = 0;
    float value = 0;

    gz = BMI088_read16(GYRO, BMI088_GYRO_RATE_Z_LSB);

    value = (int16_t)gz;
    value = gyroRange * value / 32768;

    return value;
}

int16_t BMI088_getTemperature(void) {
    uint16_t data = 0;

    data = BMI088_read16(ACC, BMI088_ACC_TEMP_MSB);
    data = data >> 5;

    if (data > 1023) {
        data = data - 2048;
    }

    return (int16_t)(data / 8 + 23);
}

void BMI088_write8(device_type_t dev, uint8_t reg, uint8_t val) {

	reg &= 0x7F;
    if (dev) {

        	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_RESET);

        	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
        	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &val, 1, 1000);

        	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_SET);
            //addr = devAddrGyro;
        } else {
        	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_RESET);

        	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
        	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &val, 1, 1000);

        	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_SET);
            //addr = devAddrAcc;
        }

//    Wire.beginTransmission(addr);
//    Wire.write(reg);
//    Wire.write(val);
//    Wire.endTransmission();

}

uint8_t BMI088_read8(device_type_t dev, uint8_t reg) {
    uint8_t data = 0, dummy = 0;

    reg |= 0x80;

    if (dev) {
    	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_RESET);

    	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
    	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &data, 1, 1000);
    	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_SET);
        //addr = devAddrGyro;
    } else {
    	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_RESET);

    	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
    	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &dummy, 1, 1000);
    	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &data, 1, 1000);
    	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_SET);
        //addr = devAddrAcc;
    }


//    Wire.beginTransmission(addr);
//    Wire.write(reg);
//    Wire.endTransmission();
//
//    Wire.requestFrom(addr, 1);
//    while (Wire.available()) {
//        data = Wire.read();
//    }

    return data;
}

uint16_t BMI088_read16(device_type_t dev, uint8_t reg) {
    uint8_t acc_buf[2];
    uint16_t msb = 0, lsb = 0;

    reg |= 0x80;

    if (dev) {
    	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_RESET);

    	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
    	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &lsb, 1, 1000);
    	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &msb, 1, 1000);
    	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_SET);
    	//addr = devAddrGyro;
    } else {
    	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_RESET);

    	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
    	HAL_SPI_Receive(&BMI088_SPI_PORT, acc_buf, 2, 1000);
    	lsb = acc_buf[1];
    	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &msb, 1, 1000);
    	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_SET);
    	//addr = devAddrAcc;
    }

//    Wire.beginTransmission(addr);
//    Wire.write(reg);
//    Wire.endTransmission();
//
//    Wire.requestFrom(addr, 2);
//    while (Wire.available()) {
//        lsb = Wire.read();
//        msb = Wire.read();
//    }

    return (lsb | (msb << 8));
}

uint32_t BMI088_read24(device_type_t dev, uint8_t reg) {
	uint8_t acc_buf[2];
    uint32_t hsb = 0, msb = 0, lsb = 0;

    reg |= 0x80;

    if (dev) {
        	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_RESET);

        	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
        	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &lsb, 1, 1000);
        	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &msb, 1, 1000);
        	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &msb, 1, 1000);
        	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_SET);
        	//addr = devAddrGyro;
        } else {
        	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_RESET);

        	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
        	HAL_SPI_Receive(&BMI088_SPI_PORT, acc_buf, 2, 1000);
        	lsb = acc_buf[1];
        	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &msb, 1, 1000);
        	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &msb, 1, 1000);
        	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_SET);
        	//addr = devAddrAcc;
        }

//    Wire.beginTransmission(addr);
//    Wire.write(reg);
//    Wire.endTransmission();
//
//    Wire.requestFrom(addr, 3);
//    while (Wire.available()) {
//        lsb = Wire.read();
//        msb = Wire.read();
//        hsb = Wire.read();
//    }

    return (lsb | (msb << 8) | (hsb << 16));
}

void BMI088_read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len) {
    uint8_t dummy = 0;

    reg |= 0x80;

    if (dev) {
       	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_RESET);

       	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
       	HAL_SPI_Receive(&BMI088_SPI_PORT, buf, len, 1000);
       	HAL_GPIO_WritePin(BMI088_CSB2_G_PORT, BMI088_CSB2_G_PIN, GPIO_PIN_SET);
           //addr = devAddrGyro;
       } else {
       	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_RESET);

       	HAL_SPI_Transmit(&BMI088_SPI_PORT, (uint8_t *) &reg, 1, 1000);
       	HAL_SPI_Receive(&BMI088_SPI_PORT, (uint8_t *) &dummy, 1, 1000);
       	HAL_SPI_Receive(&BMI088_SPI_PORT, buf, len, 1000);
       	HAL_GPIO_WritePin(BMI088_CSB1_A_PORT, BMI088_CSB1_A_PIN, GPIO_PIN_SET);
           //addr = devAddrAcc;
       }
//    Wire.beginTransmission(addr);
//    Wire.write(reg);
//    Wire.endTransmission();
//
//    Wire.requestFrom(addr, len);
//    while (Wire.available()) {
//        for (uint16_t i = 0; i < len; i ++) {
//            buf[i] = Wire.read();
//        }
//    }
}
