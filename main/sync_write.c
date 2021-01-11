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

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//



#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <time.h>

#include <esp32/rom/ets_sys.h>      // best usec delay function'
#include <esp_timer.h> // timing to find out wtf


#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address -- YOU MUST LOOK UP YOUR MODEL
//#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
//#define ADDR_PRO_GOAL_POSITION          596
//#define ADDR_PRO_PRESENT_POSITION       611

// XM540-W270
// https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
#define ADDR_PRO_TORQUE_ENABLE          (64)                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          (116)
#define ADDR_PRO_PRESENT_POSITION       (132)
#define ADDR_PRO_PRESENT_CURRENT        (126)

#define LEN_PRO_GOAL_POSITION           (4)
#define LEN_PRO_PRESENT_POSITION        (4)
#define LEN_PRO_PRESENT_CURRENT         (2)

#define ADDR_PRO_PING                   (1)
#define ADDR_PRO_REBOOT                 (8)
#define ADDR_PRO_RETURN_DELAY_TIME      (9)

// These also need to be looked up
#define DXL_MINIMUM_POSITION_VALUE      (0)             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      (4096)              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)



// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID_1                        (1)
#define DXL_ID_2                        (2)                   // Dynamixel ID: 1
//#define BAUDRATE                        115200
#define BAUDRATE                        4000000
#define DEVICENAME                      "UART1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_MOVING_STATUS_THRESHOLD     50                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define COMMAND_RETRY                   (400)     // number of times to retry

// for some reason this isn't in the header
void action2(int port_num, uint8_t id);


// wrapper for a sleep function

void sleep_usecs(int usec ) {

  ets_delay_us(usec);

}

// Have my doubes about clock_gettime--- switch to esp_timer_get_time which has shown reliable?
#if 0
char *fill_time_buf(char *buf, int buf_len) {
  struct timespec tp;
  clock_gettime(CLOCK_MONOTONIC, &tp);
  snprintf(buf, buf_len, "%ld.%06ld",tp.tv_sec,tp.tv_nsec/1000);
  return(buf);
}
#endif

char *fill_time_buf(char *buf, int buf_len) {
  uint64_t now = esp_timer_get_time();
  uint32_t secs = now / 1000000;
  uint32_t usecs = now % 1000000;
  snprintf(buf, buf_len, "%u.%06u",secs,usecs);
  return(buf);
}

// #define CHECK_GOAL 1

int set_goal_position(int port_num, uint32_t goal_pos) {

  int dxl_comm_result;  
  int dxl_error;
  uint8_t dxl_addparam_result = false;

  uint64_t t1 = esp_timer_get_time();

  // initalizae GroupSyncWrite Structs
  int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  uint64_t t2 = esp_timer_get_time();

  dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_ID_1, goal_pos, LEN_PRO_GOAL_POSITION);
  if (dxl_addparam_result != true) {
    printf("[ID:%03d] groupSyncWrite addparam failed",DXL_ID_1);
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }

  dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_ID_2, goal_pos, LEN_PRO_GOAL_POSITION);
  if (dxl_addparam_result != true) {
    printf("[ID:%03d] groupSyncWrite addparam failed",DXL_ID_2);
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }

  uint64_t t3 = esp_timer_get_time();

  groupSyncWriteTxPacket(groupwrite_num);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
    printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }

  uint64_t t4 = esp_timer_get_time();

  groupSyncWriteClearParam(groupwrite_num);

  uint64_t t5 = esp_timer_get_time();

  printf("setGoal: goal_pos %d t1~t2: %llu t2~t3 %llu t3~t4 %llu t4~t5 %llu t1~t5 %llu\n",goal_pos,t2-t1,t3-t2,t4-t3,t5-t4,t5-t1);

#ifndef CHECK_GOAL
  return(0);
#else
  int attempts = 0;
  uint32_t goal_pos_res;
  do {

    attempts++;

    printf("Writing goal position: %ud attempt %d\n",goal_pos,attempts);

    //sleep_usecs(50000);

    // check that goal  is at the right place?
    goal_pos_res = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      continue;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      continue;
    }
//    else
//    {
//      printf("read back torque, is: %d needs %d\n",torque_result,(int)enable);
//    }

    if (goal_pos_res == goal_pos) {
      printf("  goal position verified: %03d at: %s %d attempts\n",goal_pos,fill_time_buf(time_buf,sizeof(time_buf)),attempts);
      return(0);
    }
    else {
       printf("Writing goal position: %"PRIu32" response %"PRIu32" attempt %d\n",goal_pos,goal_pos_res, attempts);
    }
 
  } while(attempts < COMMAND_RETRY);

  printf("Attempted to set goal position, took more than retry, failing\n");
  return(-1);
#endif
}



int get_goal_position(int port_num, uint32_t *goal_pos_1, uint32_t *goal_pos_2) {

  int dxl_comm_result;  
  int dxl_error;
  uint8_t dxl_addparam_result = False;

  uint64_t t1 = esp_timer_get_time();

  // initalizae GroupSyncWrite Structs
  int groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  uint64_t t2 = esp_timer_get_time();

  dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID_1);
  if (dxl_addparam_result != true) {
    printf("[ID:%03d] groupSyncRead addparam failed",DXL_ID_1);
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

  dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID_2);
  if (dxl_addparam_result != true) {
    printf("[ID:%03d] groupSyncRead addparam failed",DXL_ID_2);
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

  uint64_t t3 = esp_timer_get_time();

  // do the deed
  groupSyncReadTxRxPacket(groupread_num);

  uint64_t t4 = esp_timer_get_time();

  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
    printf("SyncRead: comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("SyncRead: Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

    // Check if groupsyncread data of Dynamixel#1 is available
  uint8_t dataIsAvailable = groupSyncReadIsAvailable(groupread_num, DXL_ID_1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dataIsAvailable == False)
  {
    printf("[ID:%03d] groupSyncRead getdata not available\n", DXL_ID_1);
    groupSyncReadClearParam(groupread_num);
    printf("getGoalPosition: failing read is not available\n");
    return(-1);
  }

  // Check if groupsyncread data of Dynamixel#2 is available
  if (False == groupSyncReadIsAvailable(groupread_num, DXL_ID_2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION))
  {
    printf("[ID:%03d] groupSyncRead getdata failed\n", DXL_ID_2);
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

  uint64_t t5 = esp_timer_get_time();

  // Get Dynamixel#1 present position value
  *goal_pos_1 = groupSyncReadGetData(groupread_num, DXL_ID_1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  // Get Dynamixel#2 present position value
  *goal_pos_2 = groupSyncReadGetData(groupread_num, DXL_ID_2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  uint64_t t6 = esp_timer_get_time();

  groupSyncReadClearParam(groupread_num);

  uint64_t t7 = esp_timer_get_time();

  printf("  getGoal: pos1 %u pos2 %u t1~t2: %llu t2~t3 %llu t3~t4 %llu t4~t5 %llu t5~t6 %llu t6~t7 %llu t1~t7 %llu\n",
    *goal_pos_1, *goal_pos_2, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5, t7-t6, t7-t1);


  return(0);

}

//
// It's very important you're in the right mode, otherwise lots of things
// don't work right
//

int set_torque_mode(int port_num, int id, bool enable) {
  uint8_t torque_result;
  int attempts = 0;
  int dxl_comm_result;  
  int dxl_error;

  do {

    sleep_usecs(5000);

    attempts++;
    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE, 
        enable ? TORQUE_ENABLE : TORQUE_DISABLE);

    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      continue;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      continue;
    }
//    else
//    {
//      printf("set torque to %s\n",enable? "ENABLED" : "DISABLED");
//    }

    sleep_usecs(5000);

    // check that torque is in the correct state
    torque_result = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      continue;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      continue;
    }
//    else
//    {
//      printf("read back torque, is: %d needs %d\n",torque_result,(int)enable);
//    }

    if (enable && torque_result == TORQUE_ENABLE) {
      printf("Enabled torque, took %d attempts\n",attempts);
      return(0);
    }
    if (!enable && torque_result == TORQUE_DISABLE) {
      printf("Disabled torque, took %d attempts\n",attempts);
      return(0);
    }
 
  } while(attempts < COMMAND_RETRY);

  printf("Attempted to set torque more times than retry standard, failing\n");
  return(-1);
}

int servo_init(int port_num, int id) {
  uint8_t dxl_error = 0;    
  int dxl_comm_result = COMM_TX_FAIL;    

  printf("servo init: ID %d\n",id);

  sleep_usecs(10000);

  reboot(port_num, PROTOCOL_VERSION,id);

  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
      printf("reboot comm id %d failed: %s\n",id, getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      return(-1);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
      printf("reboot Error: id %d %s\n", id,getRxPacketError(PROTOCOL_VERSION, dxl_error));
      return(-1);
  }

  sleep_usecs(150000);

  printf("Enable torque\n");
  if (0 != set_torque_mode(port_num,id, true)) {
    printf("Enable Torque Failure: Error: id %d %s\n", id,getRxPacketError(PROTOCOL_VERSION, dxl_error));
    // coulnd't enable... disabel??
    return(-1);
  }

  return(0);
}


// ESP-IDF has app_main not main

int app_main()
{

  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  //char time_buf[60];

  int index = 0;

//  int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position
  int step_size = 100;
  int n_goals = DXL_MAXIMUM_POSITION_VALUE / step_size;
  int *dxl_goal_position = malloc((n_goals + 1) * sizeof(int));
  for (int i = 0; i < n_goals; i++) {
    dxl_goal_position[i] = i * step_size;
  }
  dxl_goal_position[n_goals++] = DXL_MAXIMUM_POSITION_VALUE;


  // set the baud rate before opening
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return 0;
  }

  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return 0;
  }


  // Initialize PacketHandler Structs
 // According to this, at least in java, you have to call this after the openport and the baudrate
 //https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/231
  packetHandler();

  // reboot

  if (0 != servo_init(port_num, DXL_ID_1)) { return(-1); }

  if (0 != servo_init(port_num, DXL_ID_2)) { return(-1); }

  printf("Start moving: \n");

  bool arrived_1, arrived_2;

  while (index < n_goals)
  {
    //printf("Press any key to continue! (or press ESC to quit!)\n");
    //if (getch() == ESC_ASCII_VALUE)
    //  break;

    // Write goal position
    //printf("  goal position start: %03d at: %s\n",dxl_goal_position[index],fill_time_buf(time_buf,sizeof(time_buf)));

    if (0 != set_goal_position(port_num, dxl_goal_position[index]) ) {
      return(-1);
    }

//    sleep_usecs(1000000);


    //printf("  goal position end: %03d at: %s\n",dxl_goal_position[index],fill_time_buf(time_buf,sizeof(time_buf)));

    // read position until goal met
    int attempts = 0;
    do
    {
      //sleep_usecs(100);

      // Read present positions
      uint32_t goal_1 = UINT32_MAX;
      uint32_t goal_2 = UINT32_MAX;
      if (0 != get_goal_position(port_num, &goal_1, &goal_2)) {
        printf("GetGoalPosition: failed exiting\n");
        return(-1);
      }

      printf(" getGoalPosition: pos 1 %u pos 2 %u goal %u\n",goal_1,goal_2,dxl_goal_position[index]);

      arrived_1 = abs(dxl_goal_position[index] - goal_1) < DXL_MOVING_STATUS_THRESHOLD;
      arrived_2 = abs(dxl_goal_position[index] - goal_2) < DXL_MOVING_STATUS_THRESHOLD;

      //if (attempts % 20 == 0) {
      //  printf("[ID:%03d] GoalPos:%03d  PresPos:%03d at %s\n", DXL_ID, dxl_goal_position[index], dxl_present_position, 
      //      fill_time_buf(time_buf, sizeof(time_buf)));
      //}

      attempts++;

    } while (( arrived_1 == false) || (arrived_2 == false));

    //printf(" achieved goal %03d at %s after %d reads\n",dxl_present_position,fill_time_buf(time_buf,sizeof(time_buf)),attempts);

    // Change goal position
    index++;
  }

//DISABLE_TORQUE:

  //set_torque_mode(port_num, DXL_ID_1, false);
  //set_torque_mode(port_num, DXL_ID_2, false);

  // Close port
  closePort(port_num);

  return 0;
}
