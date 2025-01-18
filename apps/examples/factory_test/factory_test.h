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



typedef struct packet_node_s{
	char src_addr;
	char dest_addr;
	char ack;
	unsigned int data;
	unsigned int checksum;
    unsigned int header;
	char packet_size;
} packet_node;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HEADER 0x5A
#define HEADER_SIZE 2   //In bytes
#define PACKET_SIZE 1   //In bytes
#define CHECKSUM_SIZE 2 //In bytes
#define ADDR_ACK_SIZE 3 //In bytes

#define LCD_DEV_PATH "/dev/lcd%d"

#ifndef CONFIG_FACTORY_TEST_UART_PORT
#define CONFIG_FACTORY_TEST_UART_PORT 1
#endif

packet_node Front();
packet_node Rear();
bool isEmpty();
bool isFull();
bool enQueue(packet_node value);
packet_node deQueue();
