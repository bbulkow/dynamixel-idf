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

#include <stdint.h>

#include "port_handler.h"
#include "port_handler_esp_idf.h"

int     portHandler         (const char *port_name) { return portHandlerEspIdf(port_name); }

uint8_t openPort            (int port_num) { return openPortEspIdf(port_num); }
void    closePort           (int port_num) { closePortEspIdf(port_num); }
void    clearPort           (int port_num) { clearPortEspIdf(port_num); }

void    setPortName         (int port_num, const char *port_name) { setPortNameEspIdf(port_num, port_name); }
char   *getPortName         (int port_num) { return getPortNameEspIdf(port_num); }

uint8_t setBaudRate         (int port_num, const int baudrate) { return setBaudRateEspIdf(port_num, baudrate); }
int     getBaudRate         (int port_num) { return getBaudRateEspIdf(port_num); }

int     getBytesAvailable   (int port_num) { return getBytesAvailableEspIdf(port_num); }

int     readPort            (int port_num, uint8_t *packet, int length) { return readPortEspIdf(port_num, packet, length); }
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortEspIdf(port_num, packet, length); }

void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutEspIdf(port_num, packet_length); }
void    setPacketTimeoutMSec(int port_num, double msec) { setPacketTimeoutMSecEspIdf(port_num, msec); }
uint8_t isPacketTimeout     (int port_num) { return isPacketTimeoutEspIdf(port_num); }

