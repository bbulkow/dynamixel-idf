/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_C_H_


#ifdef __GNUC__
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED
#endif

#include "robotis_def.h"

static const int DEFAULT_BAUDRATE = 57600;

int     g_used_port_num;
uint8_t *g_is_using;

int     portHandler             (const char *port_name);

uint8_t openPort                (int port_num);
void    closePort               (int port_num);
void    clearPort               (int port_num);

void    setPortName             (int port_num, const char* port_name);
char   *getPortName             (int port_num);

uint8_t setBaudRate             (int port_num, const int baudrate);
int     getBaudRate             (int port_num);

int     getBytesAvailable       (int port_num);

int     readPort                (int port_num, uint8_t *packet, int length);
int     writePort               (int port_num, uint8_t *packet, int length);

void    setPacketTimeout        (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSec    (int port_num, double msec);
uint8_t isPacketTimeout         (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_C_H_ */
