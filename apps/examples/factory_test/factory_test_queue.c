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

#include <stdbool.h>
#include "factory_test.h"
#define MAX_QUEUE_SIZE 5

packet_node queue[MAX_QUEUE_SIZE];
int front = -1;
int rear = -1;

packet_node Front() {
    if (front == -1) {
        return;
    }
    return queue[front];
}

packet_node Rear() {
    if (rear == -1) {
        return;
    }
    return queue[rear];
}

bool isEmpty() {
    return front == -1;
}

bool isFull() {
    return (rear + 1) % MAX_QUEUE_SIZE == front;
}

bool enQueue(packet_node value) {
    if ((rear + 1) % MAX_QUEUE_SIZE == front) {
        front = (front + 1) % MAX_QUEUE_SIZE;
    }
    if (front == -1) {
        front = rear = 0;
    } else {
        rear = (rear + 1) % MAX_QUEUE_SIZE;
    }
    queue[rear] = value;
    return true;
}

packet_node deQueue() {
    if (isEmpty()) {
        return;
    }
    packet_node removedElement = queue[front];
    if (front == rear) {
        front = rear = -1;
    } else {
        front = (front + 1) % MAX_QUEUE_SIZE;
    }
    return removedElement;
}
