/*******************************************************************************
* Copyright 2021 Brian Bulkowski brian@bulkowski.org
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


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/uart_types.h"
#include "hal/uart_hal.h"
#include "soc/uart_periph.h"

#include <esp32/rom/ets_sys.h> // microsecond delay

#include "esp_log.h"
#include "sdkconfig.h"

#include <esp_heap_caps.h>

// This implements the Dynamix low level interface to the ESP-IDF UART communication system.
// ESP-IDF is the RTOS used with ESP32 and related SOCs, and includes specific interfaces
// for hardware-accellerated UARTs.
// The UART documentation is here: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
//
// This code will be targeted at ESP-IDF 4.2 and should work within the 4.x ESP-IDF system.
//

#include "port_handler_esp_idf.h"

#define LATENCY_TIMER  16  // msec (USB latency timer)
                           // You should adjust the latency timer value. From the version Ubuntu 16.04.2, the default latency timer of the usb serial is '16 msec'.
                           // When you are going to use sync / bulk read, the latency timer should be loosen.
                           // the lower latency timer value, the faster communication speed.

                           // Note:
                           // You can check its value by:
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // If you think that the communication is too slow, type following after plugging the usb in to change the latency timer
                           //
                           // Method 1. Type following (you should do this everytime when the usb once was plugged out or the connection was dropped)
                           // $ echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // Method 2. If you want to set it as be done automatically, and don't want to do above everytime, make rules file in /etc/udev/rules.d/. For example,
                           // $ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
                           // $ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
                           // $ sudo udevadm control --reload-rules
                           // $ sudo udevadm trigger --action=add
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // or if you have another good idea that can be an alternatives,
                           // please give us advice via github issue https://github.com/ROBOTIS-GIT/DynamixelSDK/issues


/**
 * This is a example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
*/
#define TAG "DYNAMIXEL_SDK"



typedef struct
{
  bool    inuse;
  bool    opened;
  char    port_name[10];

  // ESP-IDF 
  int   uart_num;
  int   tx_pin;
  int   rx_pin;
  int   en_pin;
  int     baudrate;

  // esp_timer_get_time() is the best esp-idf timer, in microseconds
  int64_t  packet_start_time;  // MICROSECS
  int64_t  packet_timeout;    // MICROSECS
  int64_t  tx_time_per_byte; // NANOSECS
} PortData;

static PortData *portData = 0;


inline void IRAM_ATTR esp_delay_us(uint32_t delay) {
  uint64_t t0 = esp_timer_get_time();
  while ( (esp_timer_get_time() - t0) < delay) {
    ;
  }
}


// allocates or constructs a portHandler (integer)
//
// Simple structure: port_num 0 is 0, 1 is 1, 2 is 2, don't have any more so don't get fancier
//
// UART0 , UART1, UART2 are the valid strings


int portHandlerEspIdf(const char *port_name)
{
  int port_num;

  if (0 == strcmp("UART0", port_name)) {
    port_num = 0;
  }
  else if (0 == strcmp("UART1", port_name)) {
    port_num = 1;
  }
  else if (0 == strcmp("UART2", port_name)) {
    port_num = 2;
  }
  else {
    ESP_LOGI(TAG, "PortHandler: port name %s is invalid",port_name);
    return(-1);
  }

  // first time
  if (portData == NULL)
  {
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Start Dynamixel Library for ESP-IDF");

    portData = malloc(PORT_HANDLER_ESP_IDF_UART_MAX * sizeof(PortData));
    if (!portData) { ESP_LOGI(TAG, "PortHandler: malloc failure"); return(-1); }

    for (int i=0;i < PORT_HANDLER_ESP_IDF_UART_MAX; i++) {
      portData[i].inuse = false;
    }

    g_used_port_num = PORT_HANDLER_ESP_IDF_UART_MAX;
  }

  PortData *pd = &portData[port_num];

  if (pd->inuse) {
    ESP_LOGI(TAG, "attempted to open port %s twice, failing",port_name);
    return(-1);
  }

  switch(port_num) {
    case 0:
      pd->tx_pin = PORT_HANDLER_ESP_IDF_UART0_TXPIN;
      pd->rx_pin = PORT_HANDLER_ESP_IDF_UART0_RXPIN;
      pd->en_pin = PORT_HANDLER_ESP_IDF_UART0_ENPIN;
      break;

    case 1:
      pd->tx_pin = PORT_HANDLER_ESP_IDF_UART1_TXPIN;
      pd->rx_pin = PORT_HANDLER_ESP_IDF_UART1_RXPIN;
      pd->en_pin = PORT_HANDLER_ESP_IDF_UART1_ENPIN;
      break;

    case 2:
      pd->tx_pin = PORT_HANDLER_ESP_IDF_UART2_TXPIN;
      pd->rx_pin = PORT_HANDLER_ESP_IDF_UART2_RXPIN;
      pd->en_pin = PORT_HANDLER_ESP_IDF_UART2_ENPIN;
      break;

    default:
      ESP_LOGI(TAG, "attempted to open port %s twice, failing",port_name);
      return(-1);
  }

  pd->uart_num = port_num;
  strncpy(&pd->port_name[0],port_name,sizeof(pd->port_name)-1);
  pd->opened = false;
  pd->inuse = true;

  pd->baudrate = DEFAULT_BAUDRATE;
  pd->packet_start_time = 0;
  pd->packet_timeout = 0;
  pd->tx_time_per_byte = 0;

  return port_num;
}

uint8_t openPortEspIdf(int port_num)
{
  return( setupPortEspIdf(port_num ) );

}

void closePortEspIdf(int port_num)
{

  if (portData[port_num].inuse)
  {
    if (portData[port_num].opened) {
      ESP_ERROR_CHECK( uart_driver_delete(portData[port_num].uart_num) );
      portData[port_num].opened = false;
    }

    ESP_LOGI(TAG, "Close Port EspIdf: delete driver: uart %d",portData[port_num].uart_num);
    ESP_ERROR_CHECK( uart_driver_delete(portData[port_num].uart_num) );
    portData[port_num].inuse = false;
  }
}

// clear all INPUT bytes from the queue
void clearPortEspIdf(int port_num)
{
  uart_flush( portData[port_num].uart_num );
}

// janky
char *getPortNameEspIdf(int port_num)
{
  return portData[port_num].port_name;
}

//
// Overloading 'setBaudRate' with setupPort is poor form

uint8_t setBaudRateEspIdf(int port_num, const int baudrate)
{
  if (portData[port_num].opened) {
    ESP_LOGI(TAG, " do not set baud rate after opening, close first ");
    return False;
  }

  portData[port_num].baudrate = baudrate;

  return True;
}

int getBaudRateEspIdf(int port_num)
{
  return portData[port_num].baudrate;
}

int getBytesAvailableEspIdf(int port_num)
{
  size_t bytes_available;
  if (ESP_OK != uart_get_buffered_data_len(portData[port_num].uart_num, &bytes_available)) {
    ESP_LOGI(TAG, "Error on uarg_get_buffered_data_len");
    bytes_available = 0;
  }
  return bytes_available;
}

// Since ESP-IDF supports a wait with timeout, we could wait at least a small amount
// of time ( a few ticks ) hoping to receive the full buffer. This is almost certainly
// going to be better than tripping up to the upper level?
//
// NB: implement with a straight read and no timeout, consider improving later :-/
int IRAM_ATTR readPortEspIdf(int port_num, uint8_t *packet, int length)
{
#if 0
  size_t read_len;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(portData[port_num].uart_num, &read_len));
  if (read_len == 0) {
    ESP_LOGI(TAG, " readPortEspIdf: no bytes to read\n");
    return(0);
  }
  if (read_len > length) read_len = length;
#endif

  //uint64_t t_start = esp_timer_get_time();

  int read_len = uart_read_bytes(portData[port_num].uart_num, packet, length, 150 /*ticks to wait*/);
  if (read_len < 0) {
    // not sure what right here
    ESP_LOGI(TAG, " readPortEspIdf: uart_read_bytes returned error, clamped to 0");
    read_len = 0;
  }

//  uint64_t t_end = esp_timer_get_time();
//  printf("readPort: len_read %d t_delta %llu\n",read_len,t_end-t_start);

  return(read_len);
}

volatile int g_delay_volatile = 0;

int IRAM_ATTR writePortEspIdf(int port_num, uint8_t *packet, int length)
{

//  uint32_t cc_start = __clock_cycles();
//  uint64_t t_start = esp_timer_get_time();

  /* enable while transmitting */
  gpio_set_level(portData[port_num].en_pin, 1);

  // note: this copies to the Tx FIFO buffer, it doesn't send it - that should be OK
  // other choices are uart_wait_tx_done(), which waits until the TX buff is drained,
  // or uart_tx_chars(), which is non-blocking and returns the number of bytes written
  int r = uart_write_bytes(portData[port_num].uart_num, (const char*)packet, length);
  if (r < 0) {
    ESP_LOGI(TAG, "writePortEspIdf: uart_write_bytes returned %d unexpected",r);
    gpio_set_level(portData[port_num].en_pin, 0);
    return(0);
  }

#if 0 // THIS WAY WORKS BUT IS SLOW
// If we set the tx buffer size to 0, then we don't have to wait?
  if ( ESP_OK != uart_wait_tx_done(portData[port_num].uart_num,  100 /*os ticks to wait*/) ) {
    ESP_LOGI(TAG, "writePortEspIdf: uart_wait_tx_done returned not ok unexpected");
    gpio_set_level(portData[port_num].en_pin, 0);
    return(0);
  }
#endif

  // Trying to speed things up by cutting out all the middlepeople and ignoring the timouts,
  // and using the smallest possible delay

// low level code to check if the output buffer is flushed.
// commenting it out because the GPIO is going too slow at 4Mb
// this works great if the Servo has a 20us delay. However, it seems we're holding
// the gpio line up about 150us longer than we need using this, which doesn't work at 4mhz with 0 delay (just by a little).
  while (false == uart_ll_is_tx_idle(UART_LL_GET_HW(portData[port_num].uart_num) ) ) {
      ets_delay_us(1);
  }

 // these just delay way too long. Not just doing a few us.
 //ets_delay_us(0);
 //esp_delay_us(1);

//  for (int i=0;i<100;i++) {
//    g_delay_volatile += 1;
//  }

  // disable when done
  gpio_set_level(portData[port_num].en_pin, 0);

//  uint32_t cc_end = __clock_cycles();
//  uint64_t t_end = esp_timer_get_time();

//  printf(" clock_cycles start: %u end: %u delta: %u\n",cc_start,cc_end,cc_end-cc_start);
//  printf(" esp_timer start: %llu end %llu delta %llu\n",t_start,t_end,t_end-t_start);

  return(r);
}

void setPacketTimeoutEspIdf(int port_num, uint16_t packet_length)
{
  portData[port_num].packet_start_time = getCurrentTimeEspIdf();
  portData[port_num].packet_timeout = ((portData[port_num].tx_time_per_byte * packet_length) / 1000) + (LATENCY_TIMER * 2.0) + 2.0;
  //portData[port_num].packet_timeout *= 2.0;
  //printf("SetPacketTimeout Bytes: %.6f\n",portData[port_num].packet_timeout);
}

void setPacketTimeoutMSecEspIdf(int port_num, double msec)
{
  portData[port_num].packet_start_time = getCurrentTimeEspIdf();
  portData[port_num].packet_timeout = msec;
  //portData[port_num].packet_timeout *= 2.0;
  //printf("SetPacketTimeout Msec: %.6f\n",portData[port_num].packet_timeout);
}

uint8_t isPacketTimeoutEspIdf(int port_num)
{
  double tss = getTimeSinceStartEspIdf(port_num);
  if (tss > portData[port_num].packet_timeout)
  {
    portData[port_num].packet_timeout = 0;
    return True;
  }
  return False;
}

// fixed point microseconds
uint64_t getCurrentTimeEspIdf()
{
  return( esp_timer_get_time() );
}

// time sinze packet start

double getTimeSinceStartEspIdf(int port_num)
{
  return ((double)(getCurrentTimeEspIdf() - portData[port_num].packet_start_time) * 1000.0);
}

//
// Note: I have arranged that port_num is uart_num

uint8_t setupPortEspIdf(int port_num)
{
  PortData *pd = &portData[port_num];

    uart_config_t uart_config = {
        .baud_rate = pd->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0, // but disabled?
        .source_clk = UART_SCLK_APB,
    };

    if (pd->opened) {
      ESP_LOGI(TAG, "setupPort called when already setup, don't do that");
      return False;
    }


    int uart_num = pd->uart_num;

    if (uart_is_driver_installed(uart_num)) {
      ESP_LOGI(TAG, "setupPort unexpected uart driver already installed");
      uart_driver_delete(uart_num);
    } 

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    // NOTE this requires config UART setting - should acutally be based on what the config settings is - TODO ESP_INTR_FLAG_IRAM

    ESP_LOGI(TAG, "setupPort setting all paramters uart %d baud %d",uart_num,uart_config.baud_rate);

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins --- before setting up the driver, since the default pins conflict with all the things
    //ESP_ERROR_CHECK(uart_set_pin(uart_num, pd->tx_pin, pd->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, pd->tx_pin, pd->rx_pin, 0, 0));

    ESP_LOGI(TAG, "setupPort calling uart driver install uart %d",uart_num);

        // Set read timeout of UART TOUT feature - this is in symbols - 0 disables feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, 0 /* timeout */));

    //ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 1024, 
    //  0 /*qsize*/, NULL /*q handler*/, 0 /*intr alloc flags*/));
    // NB - better to have no transmit buffer, so we don't have to worry about waiting for it to drain?
    //int r = uart_driver_install(uart_num, 512 /*rx buffer*/, 512 /* tx buffer */ , 0 /*qsize*/, NULL /*q handler*/, 0 /*intr alloc flags*/);
    int r = uart_driver_install(uart_num, 512 /*rx buffer*/, 0 /* tx buffer */ , 0 /*qsize*/, NULL /*q handler*/, 0 /*intr alloc flags*/);
    ESP_LOGI(TAG, "Uart Driver Install: uart num %d",uart_num);
    if (r != ESP_OK) return(False);

    // Must be set after driver is installed
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // in nanos (10**9) for accuracy
    // and 10 bits per byte because of stop
    pd->tx_time_per_byte = (1000000000 / pd->baudrate) * 10; 

    // set up the enable-dsable pin
    gpio_reset_pin(pd->en_pin);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(pd->en_pin, GPIO_MODE_OUTPUT);

    gpio_set_pull_mode(pd->en_pin, GPIO_PULLDOWN_ONLY);
    /* set low */
    gpio_set_level(pd->en_pin, 0);

    pd->opened = true;


  return True; // success is generall 0. Boooooo.
}


