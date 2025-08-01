/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
/****************************************************************************
 * examples/sensor_test/sensor_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <sys/types.h>
#include <tinyara/sensors/sensor.h>

#ifdef CONFIG_SENSOR_AIS25BA
#define IR_SENSOR_PATH "/dev/sensor0"
#define MEMS_SENSOR_PATH "/dev/sensor-mems"
#define MEMS_MQ_PATH "mems_mq"

bool g_terminate;
static mqd_t g_mems_mq;
static int mems_fd;
static struct ais25ba_buf_s **gBuffer;
int buf_size; //size of segment buffer
int g_buf_num; //segment number of ais25ba

static void mems_teardown();

int mems_sensor_init()
{
	struct mq_attr attr;
	attr.mq_maxmsg = 16;
	attr.mq_msgsize = sizeof(struct mems_sensor_msg_s);
	attr.mq_curmsgs = 0;
	attr.mq_flags = 0;
        int i;
        int ret;

        mems_fd = open(MEMS_SENSOR_PATH, O_RDWR | O_SYNC, 0666);
        if (mems_fd < 0) {
                printf("ERROR: Failed to open sensor port error:%d\n", mems_fd);
                return;
        }

        ioctl(mems_fd, SENSOR_SET_SAMPRATE, AIS25BA_SAMPLE_RATE);

        printf("val of OK: %d ERROR: %d\n", OK, ERROR);
	g_mems_mq = mq_open(MEMS_MQ_PATH, O_RDWR | O_CREAT, 0644, &attr);
	if (g_mems_mq == NULL) {
		printf("ERROR: Sensor, mq_open failed!!\n");
                mems_sensor_destroy();
		return ERROR;
	}

        printf("%d\n", __LINE__);
	ret = ioctl(mems_fd, SENSOR_GET_BUFSIZE, (unsigned long)&buf_size);
	if (ret < 0) {
		printf("ERROR: Sensor, get Buffer size failed. errno : %d\n", errno);
		goto error_with_fd;
	}
        printf("%d\n", __LINE__);
	ret = ioctl(mems_fd, SENSOR_GET_BUFNUM, (unsigned long)&g_buf_num);
	if (ret < 0) {
		printf("ERROR: Sensor, get Buffer number failed. errno : %d\n", errno);
		goto error_with_fd;
	}

        printf("%d\n", __LINE__);
	ret = ioctl(mems_fd, SENSOR_REGISTERMQ, (unsigned long)g_mems_mq);
	if (ret < 0) {
		printf("ERROR: Sensor, register mq failed. errno : %d\n", errno);
                goto error_with_mq;
	}

        printf("returning with OK from mems_init\n");
	return OK;
error_with_mq:
printf("ERROR: %s %d\n", __func__, __LINE__);
	mq_close(MEMS_MQ_PATH);
error_with_fd:
        printf("ERROR: %s %d\n", __func__, __LINE__);
	close(mems_fd);
	for (int j = 0; j < i; j++) {
		if (gBuffer[j]) {
			free(gBuffer[j]);
			gBuffer[j] = NULL;
		}
	}
        printf("ERROR: %s %d\n", __func__, __LINE__);
	if (gBuffer) {
		free(gBuffer);
		gBuffer = NULL;
	}
        printf("ERROR: %s %d\n", __func__, __LINE__);
	return ERROR;
}

static int mems_sensor_prepare()
{
	int ret;
        printf("%d\n", __LINE__);
	/* Alloc array of pointers to gBuffers */
	gBuffer = (FAR struct ais25ba_buf_s **)malloc(g_buf_num * sizeof(FAR void *));
	if (gBuffer == NULL) {
		printf("ERROR: Sensor, Alloc gBuffer failed\n");
                return ERROR;
	}

        printf("%d\n", __LINE__);
        
	for (int i = 0; i < g_buf_num; i++) {
		gBuffer[i] = (FAR struct ais25ba_buf_s *)malloc(sizeof(FAR struct ais25ba_buf_s) * AIS25BA_BUFLENGTH);
		if (gBuffer[i] == NULL) {
			mems_teardown();
                        printf("gBuffer alloc failed\n");
                        return ERROR;
		}
	}

        printf("sendbuffer cmd value: %d\n", SENSOR_SENDBUFFER);
	/* Share Buffer with Driver */
	for (int i = 0; i < g_buf_num; i++) {
		ret = ioctl(mems_fd, SENSOR_SENDBUFFER, (unsigned long)gBuffer[i]);
		if (ret != OK) {
			printf("get Buffer failed. errno : %d\n", errno);
			mems_teardown();
			return ERROR;
		}
	}
        return OK;
}

static int mems_sensor_start()
{
        int ret;
        struct mems_sensor_msg_s msg;
        int prio;
        size_t size;

        /* Start Collect */
        printf("%d\n", __LINE__);
        ret = ioctl(mems_fd, SENSOR_START, NULL);
        printf("%d\n", __LINE__);
        if (ret < 0) {
                printf("ERROR: MEMS sensor start failed, errno: %d\n", errno);
                mems_teardown();
                return ERROR;
        }
        printf("%d\n", __LINE__);
        g_terminate = false;

        /*
        // Debug code
        struct mq_attr mq_stat;
        mq_getattr(g_mems_mq, &mq_stat);
        printf("mq_stats: maxmsg: %d, mq_msgsize: %d, mq_flags: %d, mq_curmsgs: %d\n", mq_stat.mq_maxmsg, mq_stat.mq_msgsize, mq_stat.mq_flags, mq_stat.mq_curmsgs);
        // End debug code
        */
        while (1) {
                if (g_terminate == true) {
                        continue;
                }
                printf("%d\n", __LINE__);

                size = mq_receive(g_mems_mq, (FAR char *)&msg, sizeof(msg), &prio);
                if (size != sizeof(msg)) {
                        printf("ERROR: wrong msg, size: %d, sizeofmsg: %d\n", size, sizeof(msg));
                        continue;
                }
                
                printf("%d\n", __LINE__);
                struct ais25ba_buf_s *buf = (struct ais25ba_buf_s *)msg.pData;
                sensor_data_s *buffer = (sensor_data_s *)buf->data;
                print_sensor_data(buffer);
                ret = ioctl(mems_fd, SENSOR_SENDBUFFER, (unsigned long)buf);
                if (ret != OK) {
                        printf("get Buffer failed. errno : %d\n", errno);
                        mems_teardown();
                        return ERROR;
                }
                /*
                if (msg.msgId == AIS25BA_MSG_DEQUEUE) {
                        sensor_data_s *buf = (sensor_data_s *)msg.pData;
                        print_sensor_data(buf);
                        ret = ioctl(mems_fd, SENSOR_SENDBUFFER, (unsigned long)buf);
                        if (ret != OK) {
                                printf("get Buffer failed. errno : %d\n", errno);
			        mems_teardown();
			        return ERROR;
                        }
                }*/
                printf("%d\n", __LINE__);
        }

        printf("%d\n", __LINE__);
        ret = ioctl(mems_fd, SENSOR_STOP, NULL);
        if (ret != OK) {
                printf("Error: sensor stop failed. errno : %d\n", errno);
                mems_teardown();
                return ERROR;
        }
        printf("%d\n", __LINE__);
        mems_teardown();
        printf("%d\n", __LINE__);
        return OK;
}

static int mems_sensor_stop()
{
        g_terminate = true;
        return OK;
}

static void mems_teardown()
{
	close(mems_fd);
	mq_close(MEMS_MQ_PATH);
	
	for (int i = 0; i < g_buf_num; i++) {
		if (gBuffer[i]) {
			free(gBuffer[i]);
			gBuffer[i] = NULL;
		}
	}
	if (gBuffer) {
		free(gBuffer);
		gBuffer = NULL;
	}
}

void mems_sensor_destroy()
{

}

static void show_usage(void)
{
	printf("usage: sensor <command #>\n");
	printf("Excute sensor testing or controling.\n\n");
        printf("    init    : Initialize the sensor operation\n");
	printf("    prepare : Prepare the sensor operation \n");
	printf("    start   : Start reading sensor data\n");
	printf("    stop    : Stop Reading sensor data\n");
}

void print_sensor_data(sensor_data_s *data)
{
	for (int i = 0; i < AIS25BA_BUFLENGTH; i++) {
                printf("x: %f, y: %f, z: %f\n", data[i].x, data[i].y, data[i].z);
	}
}

static void temp_read()
{
        sensor_data_s *data = (sensor_data_s *)malloc(sizeof(sensor_data_s)*64);

        mems_fd = open(MEMS_SENSOR_PATH, O_RDWR | O_SYNC, 0666);
        if (mems_fd < 0) {
                printf("ERROR: Failed to open sensor port error:%d\n", mems_fd);
                return;
        }

        ioctl(mems_fd, SENSOR_SET_SAMPRATE, AIS25BA_SAMPLE_RATE);
        printf("Read from MEMS sensor\n");
        while (true) {
                read(mems_fd, data, 2);
                print_sensor_data(data);
        }
        printf("sensor test complete \n");
        close(mems_fd);
}
#endif

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int sensor_main(int argc, char *argv[])
#endif
{
	printf("Sensor test!!\n");
#ifdef CONFIG_SENSOR_AIS25BA
        /*
	printf("Sensor test!!\n");
	temp_read();
	return 0;
        */
	if (argc <= 1 || !strncmp(argv[1], "-h", 2) || !strncmp(argv[1], "--help", 6)) {
		show_usage();
		return OK;
	}
	g_terminate = true;
        int status;

        status = mems_sensor_init();

	if (argc == 2) {
                if (!strncmp(argv[1], "prepare", 8)) {
                        printf("Calling mems_sensor_prepare\n");
			status = mems_sensor_prepare();
		} else if (!strncmp(argv[1], "start", 6)) {
			status = mems_sensor_start();
                        //temp_read();
		} else if (!strncmp(argv[1], "stop", 5)) {
			status = mems_sensor_stop();
		} else {
			show_usage();
		} 
	}
        if (status != OK) {
                printf("Sensor test %s failed\n", argv[1]);
        }

#endif
	return 0;
}
