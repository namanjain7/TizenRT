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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#ifndef CONFIG_DISABLE_POLL
#include <poll.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include "factory_test.h"
#include <tinyara/lcd/lcd_dev.h>	// For demonstration only

#define UART_DEV_PATH		"/dev/ttyS%d"

sq_queue_t cmd_queue;

static int start_test(void)
{
	printf("Start Factory Test\n");
	int fd = 0;
	char port[20] = {'\0'};
	packet_node packet;
	sprintf(port, UART_DEV_PATH, CONFIG_FACTORY_TEST_UART_PORT);
	fd = open(port, O_RDWR | O_SYNC, 0666);
	if (fd < 0) {
		printf("ERROR: Failed to open Tx UART(%d):\n", get_errno());
		return -1;
	} else {
		printf("Success: UART port: %d is open for factory test\n", CONFIG_FACTORY_TEST_UART_PORT);
	}

	do{
		//Fetch data from opened uart port
		/*
		unsigned int getheader;
		void* getheader_ptr = &getheader;
		int ret_size = read(fd, (void *)getheader_ptr, HEADER_SIZE);
		if (ret_size == HEADER_SIZE && getheader == HEADER) {
			//Header matched!
			//Get packet size
			int packet_size;
			ret_size = read(fd, (void *)&packet_size, PACKET_SIZE);
			if (PACKET_SIZE == ret_size) {
				packet.data = malloc(sizeof(packet.packet_size - HEADER_SIZE - PACKET_SIZE - CHECKSUM_SIZE - ADDR_ACK_SIZE));
			} else {
				printf("ERROR: Packet size read failed, resend full packet\n");
				continue;
			}
			
			// Get rest of the packet
			ret_size = read(fd, (void *)(&packet), packet_size);
			*/
			packet.header = 0x5A;
			packet.src_addr = 0xAA;
			packet.dest_addr = 0x00;
			packet.packet_size = 0x01;
			packet.checksum = 0xFF;
			packet.data = 0xFE;
			enQueue(packet);	//TODO: Configure
			
		}
		while(0);
}

void run_lcd_tc(packet_node cmd)
{
	int fd;
	char port[20] = {'\0'};
	sprintf(port, LCD_DEV_PATH, cmd.dest_addr);	// Here dest_addr is interpreted as port number of lcd
	fd = open(port, O_RDWR | O_SYNC, 0666);
	if (fd < 0) {
		printf("ERROR: Failed to open lcd port : %s error:%d\n", port, fd);
		return;	
	}

	// Respective TC ioctl will be called here.
	// For demonstration, I am manually making a frame and calling lcd ioctl
	int len = CONFIG_LCD_XRES * CONFIG_LCD_YRES * 2 + 1;
	struct lcddev_area_s area;
	area.planeno = 0;
	area.row_start = 0;
	area.row_end = CONFIG_LCD_XRES;
	area.col_start = 0;
	area.col_end = CONFIG_LCD_YRES;
	area.stride = 2 * CONFIG_LCD_XRES;
	uint8_t *lcd_data = (uint8_t *)malloc(len);
	for (int i = 0; i < len-1; i += 2) {
		lcd_data[i] = (0xF800 & 0xFF00) >> 8;
		lcd_data[i + 1] = 0xF800 & 0x00FF;
	}
	area.data = lcd_data;
	int ret = ioctl(fd, LCDDEVIO_PUTAREA, (unsigned long)(uintptr_t)&area);
	close(fd);
	sleep(1);
	free(lcd_data);

	sprintf(port, UART_DEV_PATH, CONFIG_EXAMPLES_UART_LOOPBACK_PORT);
	fd = open(port, O_RDWR | O_SYNC, 0666);
	if (fd < 0) {
		printf("ERROR: Failed to open UART:\n");
		return;
	}
	char test_data[50] = "Status: LCD test completed\n";
	write(fd, (void *)test_data, 20);
	close(fd);
}

void run_touch_tc(packet_node cmd)
{
	printf("Run lcd tc\n");
	//TODO
	return;
}

static void thread_parse_cmd()
{
	printf("Thread created for dequeue\n");
	while(true){
		if (isEmpty()) {
			continue;
		}

		packet_node cmd = deQueue();
		printf("cmd: %d\n", cmd.data);
		if (cmd.data == 0xFE) {		// Here data is signifying that test command is for LCD
			run_lcd_tc(cmd);
		}

		if(cmd.data == 0xFD) {	// Here, 0xFD is for touch test
			run_touch_tc(cmd);
		}
	}
}

/****************************************************************************
 * Public functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int factory_test_main(int argc, char **argv)
#endif
{
	int pid;
	
	pid = task_create("factory_test_cmd_thrd", SCHED_PRIORITY_DEFAULT, 8096, (main_t)thread_parse_cmd, NULL);
	pid = task_create("factory_test", SCHED_PRIORITY_DEFAULT, 8096, (main_t)start_test, NULL);
	return 0;
}
