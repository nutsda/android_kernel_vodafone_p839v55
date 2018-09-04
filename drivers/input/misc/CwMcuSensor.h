/* CWMCU.h - header file for CyWee digital 3-axis gyroscope
 *
 * Copyright (C) 2014 Cywee Motion Group Ltd.
 * Author: cywee-motion <cywee-motion@cywee.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __CWMCUSENSOR_H__
#define __CWMCUSENSOR_H__
#include <linux/ioctl.h>

#define CWMCU_I2C_NAME "CwMcuSensor"

enum ABS_status {
	CW_ABS_X = 0x01,
	CW_ABS_Y,
	CW_ABS_Z,
	CW_ABS_X1,
	CW_ABS_Y1,
	CW_ABS_Z1,
	CW_ABS_TIMEDIFF,
	CW_ABS_ACCURACY,
	//CW_ABS_TIMESTAMP_H,
	//CW_ABS_TIMESTAMP_L
	CW_ABS_TIMESTAMP,
	CW_ABS_TIMESYNC

};

enum MCU_mode {
	CW_NORMAL = 0x00,
	CW_SLEEP,
	CW_NO_SLEEP,
	CW_BOOT
};

enum MCU_status {
    CW_VAILD = 0x00,
    CW_INVAILD
};

/* power manager status */
typedef enum {
	SWITCH_POWER_ENABLE     = 0,
	SWITCH_POWER_DELAY,
	SWITCH_POWER_BATCH,
	SWITCH_POWER_NORMAL,
	SWITCH_POWER_CALIB,
	SWITCH_POWER_INTERRUPT,
	SWITCH_POWER_PCBA,
	SWITCH_POWER_POLL,
} SWITCH_POWER_ID;

/* interrupt status */
typedef enum {
	INTERRUPT_NON                   = 0,
	INTERRUPT_INIT                  = 1,
	INTERRUPT_BATCHTIMEOUT          = 2,
	INTERRUPT_BATCHFULL             = 3,
    INTERRUPT_BATCH_FLUSH           = 4,
    INTERRUPT_BATCH_TIMESTAMP_SYNC  = 5,
	INTERRUPT_DATAREADY             = 6,
    INTERRUPT_GESTURE               = 7,
    INTERRUPT_ERROR_LOG             = 8,
    INTERRUPT_DEBUG_LOG             = 9,
    INTERRUPT_TIME_SYNC             = 10,
    INTERRUPT_END					= 11,
} INTERRUPT_STATUS_LIST;

/* calibrator command */
typedef enum {
	CWMCU_CALIBRATOR_ACCELERATION   = 0,
	CWMCU_CALIBRATOR_MAGNETIC       = 1,
	CWMCU_CALIBRATOR_GYRO           = 2,
	CWMCU_CALIBRATOR_LIGHT          = 3,
	CWMCU_CALIBRATOR_PROXIMITY      = 4,
	CWMCU_CALIBRATOR_PRESSURE       = 5,
	CWMCU_CALIBRATOR_TEMPERATURE    = 6,
	CWMCU_CALIBRATOR_HUMIDITY       = 7,
	CWMCU_CALIBRATOR_STATUS         = 8,
	CWMCU_CALIBRATOR_ENABLE         = 9,
	CWMCU_CALIBRATOR_WRITE_BIAS     = 10
} CALIBRATOR_CMD;

/* firmware command */
typedef enum {
    CHANGE_TO_BOOTLOADER_MODE = 1, /*  firmware upgrade command  */
    CHANGE_TO_NORMAL_MODE     = 2, /*  firmware upgrade command  */
    MCU_RESET                 = 3, /*  firmware upgrade command  */
} FIRMWARE_CMD;

/* mcu command */
typedef enum {
    MCU_REGISTER_WRITE_CTRL     = 1,
    MCU_REGISTER_READ_CTRL      = 2,
    MCU_HWINFO_GET      		= 3,
    MCU_MAG_DEBUG	      		= 4,
    CHECK_ACC_DATA              = 7,
    CHECK_MAG_DATA              = 8,
    CHECK_GYRO_DATA             = 9,
} MCU_CMD;

/* sensor id */
typedef enum {
	CW_ACCELERATION					= 0,
	CW_MAGNETIC						= 1,
	CW_GYRO							= 2,
	CW_LIGHT						= 3,
	CW_PROXIMITY					= 4,
	CW_PRESSURE						= 5,
	CW_TEMPERATURE					= 6,
	CW_HUMIDITY						= 7,
	CW_ORIENTATION					= 8,
	CW_ROTATIONVECTOR				= 9,
	CW_LINEARACCELERATION			= 10,
	CW_GRAVITY						= 11,
	CW_STEP_COUNTER					= 12,
	CW_STEP_DETECTOR			    = 13,
	CW_MAGNETIC_UNCALIBRATED		= 14,
	CW_GYROSCOPE_UNCALIBRATED		= 15,
	CW_GAME_ROTATION_VECTOR			= 16,
	CW_GEOMAGNETIC_ROTATION_VECTOR	= 17,
	CW_SIGNIFICANT_MOTION			= 18,
	CW_SINGLE_SNAP					= 19,
	CW_DOUBLE_SNAP					= 20,
	CW_TAP							= 21,
	CW_AIR_RECOGNITION              = 22,
	CW_REAR_CAMERA                  = 24,
	CW_PICK_UP                      = 25,
	CW_FLIP                         = 26,
    CW_SCREEN_COVER                 = 27,
	CW_HAND_UP                   	= 28,
	CW_SCREEN_MIRROR                = 29,
	CW_POCKET_MODE                  = 30,
	CW_SENSORS_ID_END,
	CW_META_DATA					= 99,
	CW_MAGNETIC_UNCALIBRATED_BIAS	= 100,
	CW_GYROSCOPE_UNCALIBRATED_BIAS	= 101
} CW_SENSORS_ID;

#define CW_FWVERSION							0x3A

/*  0x01 ~ 0x0F  */
#define CW_ENABLE_REG                           0x01
#define CW_ENABLE_STATUS                        0x05
#define CW_SENSOR_DELAY_SET                     0x06    /* 2byte: byte1=>id, byte2=>ms*/
#define CW_SENSOR_DELAY_ID_SET                  0x07
#define CW_SENSOR_DELAY_GET                     0x08

#define CW_SUSPEND_ENABLE						0x0A 	/* 1 byte */
#define CW_RESUME_ENABLE						0x0B 	/* 1 byte */

#define CW_INTERRUPT_STATUS                     0x0F    /* 4Byte */

/*  0x10 ~ 0x1F  */
#define CW_BATCH_ENABLE_REG                     0x10
#define CW_BATCHENABLE_STATUS                   0x14
#define CW_BATCHTIMEOUT                         0x15    /* 5Byte ms */
#define CW_BATCHFLUSH                           0x16
#define CW_BATCHCOUNT                           0x17
#define CW_BATCHEVENT                           0x18
#define CW_BATCHTIME_SYNC                       0x19


#define	CWMCU_I2C_SENSORS_REG_START				(0x60)
#define CW_READ_ACCELERATION					(CWMCU_I2C_SENSORS_REG_START + CW_ACCELERATION)
#define CW_READ_MAGNETIC						(CWMCU_I2C_SENSORS_REG_START + CW_MAGNETIC)
#define CW_READ_GYRO							(CWMCU_I2C_SENSORS_REG_START + CW_GYRO)
#define CW_READ_LIGHT   						(CWMCU_I2C_SENSORS_REG_START + CW_LIGHT)
#define CW_READ_PROXIMITY    					(CWMCU_I2C_SENSORS_REG_START + CW_PROXIMITY)
#define CW_READ_PRESSURE	    				(CWMCU_I2C_SENSORS_REG_START + CW_PRESSURE)
#define CW_READ_ORIENTATION    					(CWMCU_I2C_SENSORS_REG_START + CW_ORIENTATION)
#define CW_READ_ROTATIONVECTOR    				(CWMCU_I2C_SENSORS_REG_START + CW_ROTATIONVECTOR)
#define CW_READ_LINEARACCELERATION				(CWMCU_I2C_SENSORS_REG_START + CW_LINEARACCELERATION)
#define CW_READ_GRAVITY   						(CWMCU_I2C_SENSORS_REG_START + CW_GRAVITY)
#define CW_READ_STEP_COUNTER					(CWMCU_I2C_SENSORS_REG_START + CW_STEP_COUNTER)
#define CW_READ_MAGNETIC_UNCALIBRATED			(CWMCU_I2C_SENSORS_REG_START + CW_MAGNETIC_UNCALIBRATED)
#define CW_READ_GYROSCOPE_UNCALIBRATED			(CWMCU_I2C_SENSORS_REG_START + CW_GYROSCOPE_UNCALIBRATED)
#define CW_READ_GAME_ROTATION_VECTOR			(CWMCU_I2C_SENSORS_REG_START + CW_GAME_ROTATION_VECTOR)
#define CW_READ_GEOMAGNETIC_ROTATION_VECTOR		(CWMCU_I2C_SENSORS_REG_START + CW_GEOMAGNETIC_ROTATION_VECTOR)

#define CW_READ_PROXIMITY_GESTURE				0x70
#define CW_READ_LIGHT_RGB						0x71
#define CW_READ_GESTURE_EVENT_COUNT				0x7A
#define CW_READ_GESTURE_EVENT_DATA				0x7B	/* read 2byte id: 1byte, data: 1byte */
#define CW_ACCURACY								0X7C


#define CW_CALIBRATOR_TYPE                      0x40
#define CW_CALIBRATOR_SENSOR_ID                 0x41
#define CW_CALIBRATOR_STATUS                    0x42
#define CW_CALIBRATOR_GET_BIAS_ACC              0x43
#define CW_CALIBRATOR_GET_BIAS_MAG              0x44
#define CW_CALIBRATOR_GET_BIAS_GYRO             0x45
#define CW_CALIBRATOR_GET_BIAS_LIGHT            0x46
#define CW_CALIBRATOR_GET_BIAS_PROXIMITY        0x47
#define CW_CALIBRATOR_GET_BIAS_PRESSURE         0x48
#define CW_CALIBRATOR_GET_BIAS_TEMPERATURE      0x49
#define CW_CALIBRATOR_GET_BIAS_HUMIDITY			0x4A
#define CW_CALIBRATOR_SET_BIAS                  0x4B
/*
Power mode control :
  Register : 0x90
  Parameter:
    PMU_SLEEP      = 0
    PMU_DEEP_SLEEP = 1
    PMU_POWERDOWN  = 2
*/

#define CW_MCU_TIMESTAMP						0X83
#define CW_MAG_DEBUG							0X84

#define CW_POWER_CTRL							0x90
#define CW_LED_CTRL								0x92

#define CW_PSENSOR_DATA_GET						0x93

#define CW_MCU_STATUS_SET						0x94
#define CW_MCU_STATUS_GET						0x95

/* mcu log */
#define CW_DEBUG_INFO_GET       				0x96
#define CW_DEBUG_COUNT_GET      				0x97
#define CW_ERROR_INFO_GET       				0x98
#define CW_ERROR_COUNT_GET      				0x99
#define CW_TEMPERATURE_RAW_DATA_GET     		0x9A    /* read, 1 byte */

#define CW_SET_STEP_COUNTER  					0x9E    /* write, 4 bytes */

#define CW_HW_SENSORLIST						0x9D

//#define CW_GYRO_TEST							0xB6

/* check data of queue if queue is empty */
#define CWMCU_NODATA							0xff

typedef enum {
	DRIVER_NO_USE            = 0,
	DRIVER_L3GD20             = 1,  //gyro
	DRIVER_LSM303DLHC    = 2,  //acc + mag
	DRIVER_LSM330             = 3,  //acc + gyro
	DRIVER_LPS331AP          = 4,  //pressure
	DRIVER_BMP280            = 5,  //pressure
	DRIVER_AKM8963          = 6,  //mag
	DRIVER_YAS53x              = 7,  //mag
	DRIVER_BMI055             = 8,  //acc + gyro
	DRIVER_AGD                   = 9,  //acc + gyro
	DRIVER_AMI                   = 10,        //mag
	DRIVER_LSM303D           = 11,        //acc + mag
	DRIVER_AKM09911                = 12,   //mag
	DRIVER_MPU6880          = 13,   //acc + gyro
	DRIVER_BMM150           = 14,   //magnetic
	DRIVER_APDS9960         = 15,        //light + proximity + gesture
	DRIVER_LSM6DS0           = 16,        //acc + gyro
	DRIVER_LP5521              = 17,
	DRIVER_HSCDTD801      = 18,        //mag
	DRIVER_LSM6DS3           = 19,        //acc + gyro
	DRIVER_BMI160             = 20,        //acc + gyro
	DRIVER_TMD2771      = 21,      //light + proximity
	DRIVER_LTR559              = 22,        //light + proximity
	DRIVER_HTS221       = 23,        //humidity + temp
	DRIVER_SHTC1               = 24,        //humidity + temp
	DRIVER_LPS25H              = 25,        //pressure
	DRIVER_STK3X1X		= 26,	//light + proximity
	DRIVER_ADPD153		= 27,	//Heart rate
	DRIVER_BH1721		= 28,	//Light
	DRIVER_GTPA200		= 29,	//pressure
} HW_ID;


#define DPS_MAX			(1 << (16 - 1))

#endif /* __CWMCUSENSOR_H__ */
