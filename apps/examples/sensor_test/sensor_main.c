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

static bool g_terminate;
static mqd_t g_mems_mq;
static int mems_fd;
static struct ais25ba_buf_s **gBuffer;
int buf_size; //size of segment buffer
int g_buf_num; //segment number of ais25ba

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

        ioctl(mems_fd, SENSOR_SET_SAMPRATE, samp);

        printf("val of OK: %d ERROR: %d\n", OK, ERROR);
	g_mems_mq = mq_open(MEMS_MQ_PATH, O_RDWR | O_CREAT, 0644, &attr);
	if (g_mems_mq == NULL) {
		printf("ERROR: Sensor, mq_open failed!!\n");
                mems_sensor_destroy();
		return ERROR;
	}

	ret = ioctl(mems_fd, SENSOR_GET_BUFSIZE, (unsigned long)&buf_size);
	if (ret < 0) {
		printf("ERROR: Sensor, get Buffer size failed. errno : %d\n", errno);
		goto error_with_fd;
	}
	ret = ioctl(mems_fd, SENSOR_GET_BUFNUM, (unsigned long)&g_buf_num);
	if (ret < 0) {
		printf("ERROR: Sensor, get Buffer number failed. errno : %d\n", errno);
		goto error_with_fd;
	}

	ret = ioctl(mems_fd, SENSOR_REGISTERMQ, (unsigned long)g_mems_mq);
	if (ret < 0) {
		printf("ERROR: Sensor, register mq failed. errno : %d\n", errno);
                goto error_with_mq;
	}

	/* Alloc array of pointers to gBuffers */
	gBuffer = (FAR struct ais25ba_buf_s **)malloc(g_buf_num * sizeof(FAR void *));
	if (gBuffer == NULL) {
		printf("ERROR: Sensor, Alloc gBuffer failed\n");
		goto error_with_fd;
	}

	for (i = 0; i < g_buf_num; i++) {
		gBuffer[i] = (FAR struct ais25ba_buf_s *)malloc(sizeof(FAR struct ais25ba_buf_s));
		if (gBuffer[i] == NULL) {
			goto error_with_fd;
		}
	}
	return OK;
error_with_mq:
	mq_close(MEMS_MQ_PATH);
error_with_fd:
	close(mems_fd);
	for (int j = 0; j < i; j++) {
		if (gBuffer[j]) {
			free(gBuffer[j]);
			gBuffer[j] = NULL;
		}
	}
	if (gBuffer) {
		free(gBuffer);
		gBuffer = NULL;
	}
	return ERROR;
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

static int mems_sensor_prepare()
{
	int ret;
	/* Share Buffer with Driver */
	for (int i = 0; i < g_buf_num; i++) {
		ret = ioctl(mFd, SENSOR_SENDBUFFER, (unsigned long)gBuffer[i]);
		if (ret != OK) {
			printf("get Buffer failed. errno : %d\n", errno);
			mems_teardown();
			return ERROR;
		}
	}
        return OK;
}

void mems_sensor_start()
{
        int ret;
        struct mems_sensor_msg_s msg;
        int prio;
        size_t size;

        /* Start Collect */
        ret = ioctl(mems_fd, SENSOR_START, NULL);
        if (ret < 0) {
                printf("ERROR: MEMS sensor start failed, errno: %d\n", errno);
                mems_teardown();
                return ERROR;
        }

        while (!g_terminate) {
                size = mq_receive(g_mems_mq, (FAR char *)&msg, sizeof(msg), &prio);
                if (size != sizeof(msg)) {
                        printf("ERROR: wrong msg, size: %d, sizeofmsg: %d\n", size, sizeof(msg));
                        continue;
                }
                
                if (msg.msgId == AIS25BA_MSG_DEQUEUE) {
                        sensor_data_s *buf = (sensor_data_s *)msg.pData;
                        print_sensor_data(buf);
                        ret = ioctl(mems_fd, SENSOR_SENDBUFFER, (unsigned long)buf);
                        if (ret != OK) {
                                printf("get Buffer failed. errno : %d\n", errno);
			        mems_teardown();
			        return ERROR;
                        }
                }
        }

        ret = ioctl(mems_fd, SENSOR_STOP, NULL);
        if (ret != OK) {
                printf("Error: sensor stop failed. errno : %d\n", errno);
                mems_teardown();
                return ERROR;
        }
        mems_teardown();
        return OK;
}

static int mems_sensor_stop()
{
        g_terminate = true;
        return OK;
}

void mems_sensor_destroy()
{

}

void print_sensor_data(sensor_data_s *data)
{
	for (int i = 0; i < 32; i++) {
                printf("x: %f, y: %f, z: %f\n", data[i].x, data[i].y, data[i].z);
	}
}

static void temp_read()
{
        int samp = 32000;
        sensor_data_s *data = (sensor_data_s *)malloc(sizeof(sensor_data_s)*64);

        mems_fd = open(MEMS_SENSOR_PATH, O_RDWR | O_SYNC, 0666);
        if (mems_fd < 0) {
                printf("ERROR: Failed to open sensor port error:%d\n", mems_fd);
                return;
        }

        ioctl(mems_fd, SENSOR_SET_SAMPRATE, samp);
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
	mems_sensor_init();
	g_terminate = false;
	if (argc == 2) {
		if (!strncmp(argv[1], "prepare", 8)) {
			return mems_sensor_prepare();
		} else if (!strncmp(argv[1], "start", 6)) {
			//return mems_sensor_start();
                        temp_read();
                        return OK;
		}/* else if (!strncmp(argv[1], "stop", 5)) {
			return mems_sensor_stop();
		}*/ else {
			show_usage();
		} 
	}
#endif
	return 0;
}
