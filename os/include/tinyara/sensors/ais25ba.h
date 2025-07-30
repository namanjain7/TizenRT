/****************************************************************************
 *
 * Copyright 2025 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_TINYARA_AIS25BA_H
#define __INCLUDE_TINYARA_AIS25BA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <tinyara/i2c.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>

#include <tinyara/i2c.h>
#include <tinyara/audio/audio.h>
#include <tinyara/audio/i2s.h>
#include <tinyara/wdog.h>
#include <tinyara/wqueue.h>

#include <tinyara/sensors/sensor.h>
#include <tinyara/sensors/ais25ba.h>

#define AIS25BA_WHOAMI_REGISTER 0x0F
#define AIS25BA_WHOAMI_VALUE 0x20
#define AIS25BA_TEST_REG 0x0B          /* Enable Self Test mode */
#define AIS25BA_TDM_CTRL_REG 0X2E      /* Control register */
#define AIS25BA_CTRL_REG_1 0x26        /* Control register, 0: Normal mode, 1: Disabled mode */      
#define AIS25BA_CTRL_REG_2 0X2F        /* Control register */
#define AIS25BA_CTRL_REG_FS 0x30       /* Accelerometer full-scale selection */
#define AIS25BA_DATA_SIZE 10		// Change according to datasheet
#define AIS25BA_ALIVECHECK_TIME 5000
#define AIS25BA_ALIVECHECK_RETRY_COUNT 3 /* Number of retry when verification fail before reinitialize sensor */
#define AIS25BA_BUFSIZE sizeof(ais25ba_buf_s)
#define AIS25BA_BUFNUM 64

/* ais25ba Message ID */
#define AIS25BA_MSG_DEQUEUE 0

#define AIS25BA_KERNEL_MQ_THREAD "iwrl6432_mq_thread"
typedef struct ais25ba_ctrl_s {
    sem_t read_sem;
    struct timespec sem_timeout;
	sem_t callback_wait_sem;
} ais25ba_ctrl_s;

typedef struct ais25ba_dev_s {
	/* I2C bus and address for device. */
	struct i2c_dev_s *i2c;
	struct i2c_config_s i2c_config;
	struct i2s_dev_s *i2s;
	/* Configuration for device. */
	struct sensor_upperhalf_s *upper;
	void *priv;
	struct ais25ba_ctrl_s ctrl;

	WDOG_ID wdog;
	struct work_s work;

	mqd_t mq;
	struct sq_queue_s pendq;	/* Queue of empty buffer */
	struct sq_queue_s doneq;	/* Queue of sensor data buffer */

} ais25ba_dev_s;

typedef struct ais25ba_buf_s {
	struct dq_entry_s entry;		 	/* double linked queue entry */
	uint16_t maxbyte;            		/* Total byte of buffer */
	uint16_t curbyte;                   /* Currently used */
	uint8_t data[AIS25BA_DATA_SIZE];   /* Actual Buffer include Header */
	uint16_t msgId;                     /* msgId to be shared */
} ais25ba_buf_s;

#endif	/* __INCLUDE_TINYARA_AIS25BA_H */ 
