/*******************************************************************************
* Copyright 2020 Brian Bulkowski
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

/* Author: Brian Bulkowski */
/* Based on code from Dynamixel */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_EspIdf_PORTHANDLEREspIdf_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_EspIdf_PORTHANDLEREspIdf_C_H_

// PIN configuration goes here (for the moment)
#define PORT_HANDLER_ESP_IDF_UART0_TXPIN	0
#define PORT_HANDLER_ESP_IDF_UART0_RXPIN	0
#define PORT_HANDLER_ESP_IDF_UART0_ENPIN	0

#define PORT_HANDLER_ESP_IDF_UART1_TXPIN	(14)  // default: 10
#define PORT_HANDLER_ESP_IDF_UART1_RXPIN	(15)   // default: 9
#define PORT_HANDLER_ESP_IDF_UART1_ENPIN	(2)

#define PORT_HANDLER_ESP_IDF_UART2_TXPIN	(17)
#define PORT_HANDLER_ESP_IDF_UART2_RXPIN	(16)
#define PORT_HANDLER_ESP_IDF_UART2_ENPIN	0

#define PORT_HANDLER_ESP_IDF_UART_MAX		3

#define PORT_HANDLER_ESP_IDF_BUF_SIZE 1024

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_system.h"

#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Need accurate time
__attribute__ ((always_inline)) inline static uint32_t __clock_cycles() {
  uint32_t cyc;
  __asm__ __volatile__ ("rsr %0,ccount":"=a" (cyc));
  return cyc;
}



#include "port_handler.h"

int portHandlerEspIdf            (const char *port_name);

uint8_t setupPortEspIdf          (int port_num);
uint8_t setCustomBaudrateEspIdf  (int port_num, int speed);
int     getCFlagBaud            (const int baudrate);

uint64_t  getCurrentTimeEspIdf     ();
double  getTimeSinceStartEspIdf  (int port_num);

uint8_t openPortEspIdf           (int port_num);
void    closePortEspIdf          (int port_num);
void    clearPortEspIdf          (int port_num);

void    setPortNameEspIdf        (int port_num, const char *port_name);
char   *getPortNameEspIdf        (int port_num);

uint8_t setBaudRateEspIdf        (int port_num, const int baudrate);
int     getBaudRateEspIdf        (int port_num);

int     getBytesAvailableEspIdf  (int port_num);

int     readPortEspIdf           (int port_num, uint8_t *packet, int length);
int     writePortEspIdf          (int port_num, uint8_t *packet, int length);

void    setPacketTimeoutEspIdf     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecEspIdf (int port_num, double msec);
uint8_t isPacketTimeoutEspIdf      (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_EspIdf_PORTHANDLEREspIdf_C_H_ */
