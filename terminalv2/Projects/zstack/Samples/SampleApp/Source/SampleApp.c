/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "mt_uart.h"//串口头文件
#include "MT_APP.H"
#include "MT.h"//CMD_SERIAL_MSG任务ID在这里定义的
#include <ioCC2530.h>
#include <stdlib.h>
/************编译协调者的时候需要将注释去掉**********/
//#define DEV_COOR
//#define DEBUG_STAGE
/*****************************/
#ifndef DEV_COOR
#include "ds18b20.H"
#endif
/**********************定义命令******************************/
#ifndef CMD_H
#define CMD_H

/***定义命令的类型CmdType****/
#define CMD_TYPE_CLOSELINK             0x00         /*断开连接*/
#define CMD_TYPE_QUERYREQUEST          0x01         /*查询请求*/
#define CMD_TYPE_QUERYRESPONSE         0x02         /*查询应答*/
#define CMD_TYPE_CONTROLREQUEST        0x03         /*控制请求*/
#define CMD_TYPE_CONTROLRESPONSE       0x04         /*控制应带*/
/***cmd命令操作类型CmdCtrlType*/
#define CMD_CONTROL_OPEN               0x00     // 打开 
#define CMD_CONTROL_CLOSE              0x01     // 关闭
#define CMD_CONTROL_WRITE              0x02     // 写 
#define CMD_CONTROL_READ               0x03     // 读 
#define CMD_CONTROL_START              0x04     // 开始 
#define CMD_CONTROL_STOP               0x05     // 结束
/*
CmdCtrlValue     0x00 - 0xff
*/
/***定义房间ID CmdRoomId***/
#define CMD_ROOM_LIVINGROOM             0x00       //客厅
#define CMD_ROOM_BEDROOM                0x01       //卧室
#define CMD_ROOM_KITCHEN                0x02       //厨房
/**cmd设备类型定义CmdDevType
  在这里先定义
  */
#define CMD_DEV_LED                0x00      // 灯
#define CMD_DEV_CURTAIN            0x01      //窗帘
#define CMD_DEV_TEMPERATURE        0x02       //温度
#define CMD_DEV_LIGHTINTENSITY     0x03       //光线强度
#define CMD_DEV_AIR                0x04       //烟雾
#define CMD_DEV_WINDOWMAGNETIC     0x05       //窗磁
#define CMD_DEV_DOORBELL           0x06       //门铃
#define CMD_DEV_OUTPUT             0x07       //输出预留
#define CMD_DEV_INPUT              0x08       //输入预留
  
/**命令的类型**/
#define  CmdFlag         0                             /*帧头*/                        //0x7f
#define  CmdType         1                             /* 命令类型 */
#define  CmdCtrlType     2                             /* 命令控制的类型 */
#define  CmdCtrlValue    3                             /* 命令值 */
#define  CmdRoomId       4                             /* 房间号 */
#define  CmdDevType      5                             /* 设备类型 */
#define  CmdDevId        6                             /* 设备号 */
/***************************************
CmdDevId 0x00 - 0xff

定义的数目模型为 7f 0X 0X 0X 0X 0X 0X
**************************************/
#define  CMD_LEN         7                 //一共是7个16位进制             
#endif
/*****************************以上为CMD******************************************/
/*********自带led定义**********/
#define LED1    HAL_LED_1  //自带的LED          控制的就是 P1.0
#define LED2    HAL_LED_2                            //   p1.1
#define LED3    HAL_LED_3                      //不要使用    P1.3         
#define LED_ON  HAL_LED_MODE_ON  //输出为高电平
#define LED_OFF HAL_LED_MODE_OFF
/*********************各设备功能模块定义************************
第一步：在这里添加外设的信息并设置相应的动作*/
/*****************************************************************
    功能实现：DEV_LIGHT          灯      P0.0   输出 
                                         P0.1   输出
                                         P0.4   输出
              DEV_CURTAIN       窗帘     P0.5   输出
              DEV_TEMPERATURE    温度    P0.6   输入
              DEV_LIGHT         光强     P0.7   输入
              DEV_IAR          烟雾      P1.2   输入
         DEV_WINDOWMAGNETIC     窗磁     P1.5   输入
            DEV_DOORBELL        门铃     P2.0   中断 
          DEV_OUTPUT            输出预留  P1.6 P1.7
          DEV_INPUT             输入预留  p2.3 p2.4
***************************************************************/
//灯 P0.4 输出
#ifdef  DEV_LIGHT
  #define DEV_LIGHT1_OPEN()        do{P0_0 = 1;}while(0);
  #define DEV_LIGHT1_CLOSE()       do{P0_0 = 0;}while(0);
  #define DEV_LIGHT1_BIT           (P0_0)
  #define DEV_LIGHT2_OPEN()        do{P0_1 = 1;}while(0);
  #define DEV_LIGHT2_CLOSE()       do{P0_1 = 0;}while(0);
  #define DEV_LIGHT2_BIT           (P0_1)
  #define DEV_LIGHT3_OPEN()        do{P0_4 = 1;}while(0);
  #define DEV_LIGHT3_CLOSE()       do{P0_4 = 0;}while(0);
  #define DEV_LIGHT3_BIT           (P0_4)
#endif
//窗帘 P0.5 输出
#ifdef  DEV_CURTAIN
  #define DEV_CURTAIN_OPEN()       do{P0_5 = 1;}while(0);
  #define DEV_CURTAIN_CLOSE()      do{P0_5 = 0;}while(0);
  #define DEV_CURTAIN_BIT          (P0_5)
#endif


 //光强 p0.7
#ifdef DEV_LIGHTINTENSITY
  #define DEV_LIGHTINTENSITY_BIT            (P0_7)
#endif
   //烟雾 p1.2
#ifdef DEV_IAR
  #define  DEV_IAR_BIT               (P1_2)
#endif
   //窗磁 p1.5
#ifdef DEV_WINDOWMAGNETIC
  #define  DEV_WINDOWMAGNETIC_BIT     (P1_5)
#endif
   //门铃 p2.0
#ifdef DEV_DOORBELL
#define DEV_DOORBELL_BIT             (P2_0)
#endif
//预留输出 p1.6 1.7
#ifdef DEV_OUTPUT
 #define DEV_OUTPUT1_OPEN()      do{P1_6 = 1;}while(0);
 #define DEV_OUTPUT1_CLOSE()      do{P1_6 = 0;}while(0);
 #define DEV_OUTPUT1_BIT            (P1_6)
 #define DEV_OUTPUT2_OPEN()      do{P1_7 = 1;}while(0);
 #define DEV_OUTPUT2_CLOSE()      do{P1_7 = 0;}while(0);
 #define DEV_OUTPUT2_BIT            (P1_7)
#endif
   //预留输入 p2.3 p2.4
#ifdef DEV_OUTPUT
 #define DEV_OUTPUT1_OPEN()      do{P1_6 = 1;}while(0);
 #define DEV_OUTPUT1_CLOSE()      do{P1_6 = 0;}while(0);
 #define DEV_OUTPUT1_BIT            (P1_6)
 #define DEV_OUTPUT2_OPEN()      do{P1_7 = 1;}while(0);
 #define DEV_OUTPUT2_CLOSE()      do{P1_7 = 0;}while(0);
 #define DEV_OUTPUT2_BIT            (P1_7)
#endif
   //预留输入 
#ifdef DEV_INPUT
   #define DEV_INPUT1_BIT            (P2_3)
   #define DEV_INPUT2_BIT            (P2_4)
#endif
/*****************功能模块定义结束**************************/

/***定义特有的房间号**/
/********************************************
    LIVING_ROOM         客厅
    BED_ROOM            卧室
    KITCHEN_ROOM        厨房
*******************************************/
#ifndef DEV_COOR
    // #error  outdev_coor
     #ifdef  LIVING_ROOM     //客厅
     #define ROOM_NUM   CMD_ROOM_LIVINGROOM //把房间号定义为客厅
     #endif
     #ifdef BED_ROOM         //卧室
       #define ROOM_NUM CMD_ROOM_BEDROOM
     #endif 
     #ifdef KITCHEN_ROOM      //厨房
       #define ROOM_NUM CMD_ROOM_KITCHEN
     #endif
     #define DEV_NUM 1
#endif
/***********************************************************/
/******************定义房间数量与设备的孰料***************/
#ifdef DEV_COOR

  //#error  indev_coor

  #define ROOM_COUNT 3
  #define DEV_COUNT 1
#endif
/********************************************************/

//定义标志东西
#define FRAME_FLAG  (0x7f)   //数据框架判断
#define ERROR_FLAG  (0x6f)   
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.





endPointDesc_t SampleApp_epDesc;
#ifndef DEV_COOR
   int DoorBellPressed = 0;//门铃
#endif



/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;
#ifdef DEBUG_STAGE          //debug模式提供ASCII编码
       uint8 asc_16[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
#endif      
uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;
/**自己定义的点对点的传输**/

  afAddrType_t SampleApp_Point_to_Point_DstAddr;

/**************/
aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
/*******路由表单********/
#ifdef DEV_COOR
  uint16 Routing_Table[ROOM_COUNT][DEV_COUNT]={0};
#endif

//unsigned char temp_bank = 0;
/************/
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );

/***my funtions**/
#ifdef DEV_COOR
  void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);//串口接受到数据处理函数的申明
#endif
  //void P1_ISR(void);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id; //任务ID
  SampleApp_NwkState = DEV_INIT;//网络类型
  SampleApp_TransID = 0;// 设置发送数据的方式和目的地址,
  /*****串口初始化******/
  MT_UartInit ();
  MT_UartRegisterTaskID(task_id);

  #ifndef DEV_COOR
    #ifdef DEBUG_STAGE
      #ifdef LIVING_ROOM
       HalUARTWrite(0,"livingroom\n",11);
      #endif
      #ifdef BED_ROOM
       HalUARTWrite(0,"bedroom\n",8);
      #endif
      #ifdef KIT_ROOM
       HalUARTWrite(0,"kitroom\n",8);
      #endif
    #endif
  #endif

  /******************终端设备各模块io初始化*******************/
  /*
       第二步：这里添加需要的设置的IO初始化
  ***********************************************************/
#ifndef DEV_COOR
    //灯 P0.0 1 4 输出
   #ifdef  DEV_LIGHT
       P0SEL &= 0xec;//灯1配置I/O口  P0_0
       P0DIR |= 0x13;//LIGHT1 输出
       P0_0 = 0;
       P0_1 = 0;
       P0_4 = 0;      
   #endif
 //窗帘 P0.5 输出  
   #ifdef DEV_CURTAIN
      P0SEL &= 0xDF;
      P0DIR |= 0x40;  //输出  
      P0_5 = 0;
   #endif
      //温度 P0.6
   #ifdef DEV_TEMPERATURE
      P0SEL &= 0xbf;         //DS18B20的io口初始化温度初始化
      Temp_test();//测试温度
   #endif
     //光强 P0.7
   #ifdef DEV_LIGHTINTENSITY
      P0SEL &= 0x7f;
      P0DIR &= 0x7f;
   #endif 
      //烟雾 P1.2
   #ifdef DEV_AIR
      P1SEL &= 0xfb;
      P1DIR &= 0xfb;  //输入
   #endif
      //窗磁 P1.5
   #ifdef DEV_WINDOWMAGNETIC
      P1SEL &= 0xef;
      P1DIR &= 0xef;
      P1INP &= 0xef;
   #endif
      //门铃 p2.0
   #ifdef DEV_DOORBELL   
  
   #endif
      //output
   #ifdef DEV_OUTPUT
      P1SEL &= 0x3f;
      P1DIR |= 0xc0;
   #endif
      //input
   #ifdef DEV_INPUT
     P2SEL  &= 0xf9;
     P2DIR  &= 0xe7;
   #endif
#endif
/************IO初始化结束**********************/
      
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
/***设定点对点的地址方案****/
  SampleApp_Point_to_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Point_to_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0xffff;//数据从终端发给协调者
/********************************/
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent 任务处理函数。任务处理函数是对任务发生后的事件进行处理
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )//发送处理函数
{
  afIncomingMSGPacket_t *MSGpkt;
  #ifndef DEV_COOR//存放加入网络的数据
     uint8 buffer[4];
  #endif
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef DEV_COOR
        case CMD_SERIAL_MSG://串口收到数据后由MT_UART层传递过来的数据，用网蜂方法接收，编译时不定义MT相关内容， /***这个函数是来自电脑的串口数据处理函数***/
           SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);//串口收到信息后，事件号 CMD_SERIAL_MSG 就会被登记，便进入 CMD_SERIAL_MSG: 执行 SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);         
           break;
#endif
            
        // Received when a key is pressed
        case KEY_CHANGE:
          //按键发送改变
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE://网络出现状态机发生变化
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SampleApp_NwkState == DEV_ZB_COORD)||//协调者不给自己点播
               //(SampleApp_NwkState == DEV_ROUTER)|| //协调者不给路由点播
              (SampleApp_NwkState == DEV_END_DEVICE))
          {
 #ifndef DEV_COOR//终端向协调器发送加入网络的数据包 数据包格式为    m 房间号（ROOM_NUM） 设备号（DEV_NUM） g
            buffer[0] = 'm';
            buffer[1] = ROOM_NUM;//          
            buffer[2] = DEV_NUM;
            buffer[3] = 'g';
            SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
            if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr,
                       &SampleApp_epDesc,
                       SAMPLEAPP_ADDNET_CLUSTERID,
                       4,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE | AF_ACK_REQUEST,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                   {
                      #ifdef DEBUG_STAGE
                               HalUARTWrite(0,"Init EndPoint\n\r",sizeof("Init EndPoint\n\r"));
                             //HalLedSet(LED1,LED_ON);
                      #endif 
                   }
                else
                  {
                  // Error occurred in request to send.
                   }
            /*****原函数被注释
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
            */
#endif
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )//加入网络后一直在执行这个状态
  {
    // Send the periodic message
    SampleApp_SendPeriodicMessage();
    /**********修改使用的函数********/
    //SampleApp_PointToPointMessage();
    /******/
    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.所有按键处理函数
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
     #ifdef DEV_DOORBELL
       DoorBellPressed =1;
     #endif
    //HalLedBlink(HAL_LED_1,2,50,500);//
   // SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    /*
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
    */
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB接收处理函数。
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )//接受数据处理函数
{
 // uint16 flashTime;
  uint8 *str = NULL; 
  #ifndef DEV_COOR
     uint8 reply_buf[CMD_LEN]; //reply_buf[7]
     #ifdef DEBUG_STAGE
       int i;
     #endif
  #endif
  #ifdef DEV_COOR
      uint8 buffer[4];//用于处理应答
      uint16 shortaddr;//用来存放短地址
  #endif
  str = pkt->cmd.Data;
  switch ( pkt->clusterId )
  {
    /***自己添加的条件分支***/
    /*****
    第三步：添加自己的状态机
    ******/
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
      #ifndef DEV_COOR                         
          #ifdef DEBUG_STAGE
             for(i=0;i<CMD_LEN;i++)
             HalUARTWrite(0,str+i,1 ); 
          #endif 
            HalLedSet(LED2,LED_ON);//终端led2亮 表示得到命令
            if(str[CmdFlag] == FRAME_FLAG && pkt->srcAddr.addr.shortAddr == 0x0000)   //确认帧头和源地址 
            {
              reply_buf[CmdFlag] = FRAME_FLAG;//框架判断
              switch(str[CmdDevType])
              {
                /************************灯******************/
                #ifdef DEV_LIGHT
                  case CMD_DEV_LED:
                    if(str[CmdType] == CMD_TYPE_CONTROLREQUEST)//是否为控制
                     {
                          switch(str[CmdCtrlType])//命令控制的类型
                          {
                                case CMD_CONTROL_OPEN:
                                              {
                                                  reply_buf[CmdCtrlValue] = 1; 
                                                  switch(str[CmdDevId])
                                                  {
                                                     case 0:
                                                        DEV_LIGHT1_OPEN()
                                                        break;
                                                     case 1:
                                                        DEV_LIGHT2_OPEN()
                                                        break;
                                                     case 2:
                                                        DEV_LIGHT3_OPEN()
                                                        break;
                                                     default:
                                                            reply_buf[CmdCtrlValue] = ERROR_FLAG;
                                                  }
                                              }
                               break;
                               case CMD_CONTROL_CLOSE:
                                            {
                                               reply_buf[CmdCtrlValue] = 0; 
                                               switch(str[CmdDevId])
                                                  {
                                                     case 0:
                                                        DEV_LIGHT1_CLOSE()
                                                        break;
                                                     case 1:
                                                        DEV_LIGHT2_CLOSE()
                                                        break;
                                                     case 2:
                                                        DEV_LIGHT3_CLOSE()
                                                        break;
                                                     default:
                                                            reply_buf[CmdCtrlValue] = ERROR_FLAG;
                                                  }
                                            }
                               break;
                               default:
                                     reply_buf[CmdCtrlType] = ERROR_FLAG;   
                          }
                          reply_buf[CmdType] = CMD_TYPE_CONTROLRESPONSE; 
                     }                  
                    else if(str[CmdType] == CMD_TYPE_CONTROLREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)//查询
                    {
                      switch(str[CmdDevId])
                      {
                        case 0:
                            if(DEV_LIGHT1_BIT)
                               reply_buf[CmdCtrlValue] = 1;
                            else
                               reply_buf[CmdCtrlValue] = 0;
                        break;
                       case 1:
                            if(DEV_LIGHT2_BIT)
                               reply_buf[CmdCtrlValue] = 1;
                            else
                               reply_buf[CmdCtrlValue] = 0;
                         break;
                       case 2:
                            if(DEV_LIGHT3_BIT)
                               reply_buf[CmdCtrlValue] = 1;
                            else
                               reply_buf[CmdCtrlValue] = 0;
                         break;
                        default:
                             reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
                    }
                    reply_buf[CmdDevType] = CMD_DEV_LED;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
             #endif
                  /**************************窗帘*************************************/
             #ifdef DEV_CURTAIN
                  case CMD_DEV_CURTAIN:
                    if(str[CmdType] == CMD_TYPE_CONTROLREQUEST)//是否为控制
                     {
                          switch(str[CmdCtrlType])//命令控制的类型
                          {
                                case CMD_CONTROL_OPEN:
                                              {
                                                  reply_buf[CmdCtrlValue] = 1; 
                                                  switch(str[CmdDevId])
                                                  {
                                                     case 0:
                                                        DEV_CURTAIN_OPEN()
                                                        break;
                                                     default:
                                                            reply_buf[CmdCtrlValue] = ERROR_FLAG;
                                                  }
                                              }
                               break;
                               case CMD_CONTROL_CLOSE:
                                            {
                                               reply_buf[CmdCtrlValue] = 0; 
                                               switch(str[CmdDevId])
                                                  {
                                                     case 0:
                                                        DEV_CURTAIN_CLOSE()
                                                        break;
                                                     default:
                                                            reply_buf[CmdCtrlValue] = ERROR_FLAG;
                                                  }
                                            }
                               break;
                               default:
                                     reply_buf[CmdCtrlType] = ERROR_FLAG;   
                          }
                          reply_buf[CmdType] = CMD_TYPE_CONTROLRESPONSE; 
                     }                  
                    else if(str[CmdType] == CMD_TYPE_CONTROLREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)//查询
                    {
                      switch(str[CmdDevId])
                      {
                        case 0:
                            if(DEV_CURTAIN_BIT)
                               reply_buf[CmdCtrlValue] = 1;
                            else
                               reply_buf[CmdCtrlValue] = 0;
                        break;
                        default:
                             reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
                    }
                    reply_buf[CmdDevType] = CMD_DEV_LED;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
             #endif
              /*******************************************温度**********************************************/
          #ifdef DEV_TEMPERATURE
           case  CMD_DEV_TEMPERATURE:
          if(str[CmdType] == CMD_TYPE_QUERYREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)
          {
           // P0SEL &= 0xbf;         //DS18B20的io口初始化
            switch(str[CmdDevId])
              {
                case 0:
                  Temp_test();
               break;
               default:
                          reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
               }
                    reply_buf[CmdDevType] = CMD_DEV_TEMPERATURE;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
         #endif
                   /*******************************************光强**********************************************/
         #ifdef DEV_LIGHTINTENSITY
                  case  CMD_DEV_LIGHTINTENSITY:
          if(str[CmdType] == CMD_TYPE_QUERYREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)
          {
         
            switch(str[CmdDevId])
              {
                case 0:
                  if(DEV_LIGHTINTENSITY_BIT)
                     reply_buf[CmdCtrlValue] = 1;
                  else
                     reply_buf[CmdCtrlValue] = 0;
                break;
               default:
                          reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
               }
                    reply_buf[CmdDevType] = CMD_DEV_LIGHTINTENSITY;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
         #endif    
                       /*******************************************烟雾**********************************************/
         #ifdef DEV_AIR
                  case  CMD_DEV_AIR:
          if(str[CmdType] == CMD_TYPE_QUERYREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)
          {
         
            switch(str[CmdDevId])
              {
                case 0:
                  if(DEV_AIR_BIT)
                     reply_buf[CmdCtrlValue] = 1;
                  else
                     reply_buf[CmdCtrlValue] = 0;
                break;
               default:
                          reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
               }
                    reply_buf[CmdDevType] = CMD_DEV_AIR;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
         #endif 
                  /******************************窗磁*************************************/
         #ifdef DEV_WINDOWMAGNETIC
            case  CMD_DEV_WINDOWMAGNETIC:
          if(str[CmdType] == CMD_TYPE_QUERYREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)
          {
         
            switch(str[CmdDevId])
              {
                case 0:
                  if(DEV_WINDOWMAGNETIC_BIT)
                     reply_buf[CmdCtrlValue] = 1;
                  else
                     reply_buf[CmdCtrlValue] = 0;
                break;
               default:
                          reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
               }
                    reply_buf[CmdDevType] = CMD_DEV_WINDOWMAGNETIC;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
         #endif
         /******************************门铃*************************************/
         #ifdef DEV_DOORBELL 
            case  CMD_DEV_DOORBELL:
          if(str[CmdType] == CMD_TYPE_QUERYREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)
          {
         
            switch(str[CmdDevId])
              {
                case 0:
                  if(DEV_DOORBELL_BIT == 0)
                     reply_buf[CmdCtrlValue] = 0;
                  else
                  {
                       reply_buf[CmdCtrlValue] = 1;
                       DoorBellPressed = 0;
                  }
                break;
               default:
                          reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
               }
                    reply_buf[CmdDevType] = CMD_DEV_DOORBELL;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
         #endif  
                   /************************输出预留******************/
                #ifdef DEV_OUTPUT
                  case CMD_DEV_OUTPUT:
                    if(str[CmdType] == CMD_TYPE_CONTROLREQUEST)//是否为控制
                     {
                          switch(str[CmdCtrlType])//命令控制的类型
                          {
                                case CMD_CONTROL_OPEN:
                                              {
                                                  reply_buf[CmdCtrlValue] = 1; 
                                                  switch(str[CmdDevId])
                                                  {
                                                     case 0:
                                                        DEV_OUTPUT1_OPEN()
                                                        break;
                                                     case 1:
                                                        DEV_OUTPUT2_OPEN()
                                                        break;
                                                     default:
                                                            reply_buf[CmdCtrlValue] = ERROR_FLAG;
                                                  }
                                              }
                               break;
                               case CMD_CONTROL_CLOSE:
                                            {
                                               reply_buf[CmdCtrlValue] = 0; 
                                               switch(str[CmdDevId])
                                                  {
                                                     case 0:
                                                        DEV_OUTPUT1_CLOSE()
                                                        break;
                                                     case 1:
                                                        DEV_OUTPUT2_CLOSE()
                                                        break;
                                                     default:
                                                            reply_buf[CmdCtrlValue] = ERROR_FLAG;
                                                  }
                                            }
                               break;
                               default:
                                     reply_buf[CmdCtrlType] = ERROR_FLAG;   
                          }
                          reply_buf[CmdType] = CMD_TYPE_CONTROLRESPONSE; 
                     }                  
                    else if(str[CmdType] == CMD_TYPE_CONTROLREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)//查询
                    {
                      switch(str[CmdDevId])
                      {
                        case 0:
                            if(DEV_OUTPUT1_BIT)
                               reply_buf[CmdCtrlValue] = 1;
                            else
                               reply_buf[CmdCtrlValue] = 0;
                        break;
                       case 1:
                            if(DEV_OUTPUT2_BIT)
                               reply_buf[CmdCtrlValue] = 1;
                            else
                               reply_buf[CmdCtrlValue] = 0;
                         break;
                      
                        default:
                             reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
                    }
                    reply_buf[CmdDevType] = CMD_DEV_OUTPUT;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
             #endif
                    /******************************输入预留*************************************/
         #ifdef DEV_INPUT
            case  CMD_DEV_INPUT:
          if(str[CmdType] == CMD_TYPE_QUERYREQUEST && str[CmdCtrlType] == CMD_CONTROL_READ)
          {
         
            switch(str[CmdDevId])
              {
                case 0:
                  if(DEV_INPUT_BIT1)
                     reply_buf[CmdCtrlValue] = 1;
                  else
                     reply_buf[CmdCtrlValue] = 0;
                break;
                case 1:
                  if(DEV_INPUT_BIT2)
                     reply_buf[CmdCtrlValue] = 1;
                  else
                     reply_buf[CmdCtrlValue] = 0;
                break;
               default:
                          reply_buf[CmdCtrlValue] = ERROR_FLAG;  
                      }
                      reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE; 
               }
                    reply_buf[CmdDevType] = CMD_DEV_INPUT;
                    reply_buf[CmdCtrlType] = str[CmdCtrlType];
                    reply_buf[CmdDevId] = str[CmdDevId];
                    reply_buf[CmdRoomId] = ROOM_NUM;
                    SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                        {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                        }
                   else
                    {
                          HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                     }
                  break;
         #endif
              default://没找到设备
                reply_buf[CmdType] = CMD_TYPE_QUERYRESPONSE;
                reply_buf[CmdCtrlType] = ERROR_FLAG;
                reply_buf[CmdCtrlValue]= ERROR_FLAG;
                reply_buf[CmdRoomId] = ROOM_NUM;
                reply_buf[CmdDevType] = str[CmdDevType];
                reply_buf[CmdDevId] = str[CmdDevId];
                SampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0x0000;
                    if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                           CMD_LEN,
                           reply_buf,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                      {
                           HalLedSet(LED2,LED_OFF);//终端led2灭 表示发送命令成功
                       }
                   else
                      {
                           HalLedSet(LED2,LED_ON);//终端led2灭 表示发送命令成功
                          // Error occurred in request to send.
                      }
                  break;
              }         
            }
      #else//协调器将由终端返回的回应信息包发送给上位机
      if(str[CmdType] == CMD_TYPE_QUERYRESPONSE || str[CmdType] == CMD_TYPE_CONTROLRESPONSE)
      {
        HalUARTWrite(0,str,CMD_LEN);
      }      
      #endif
    break;
    /*************************/
    case SAMPLEAPP_ADDNET_CLUSTERID://入网处理
      #ifdef DEV_COOR
          if(pkt->cmd.Data[0]=='m' && pkt->cmd.Data[3]=='g' && pkt->srcAddr.addr.shortAddr != 0)
          {
            HalLedSet(LED1,LED_ON);//协调者led1亮 表示找到了网络
            shortaddr = pkt->srcAddr.addr.shortAddr;//把收到的终端的地址保存
            Routing_Table[pkt->cmd.Data[1]][pkt->cmd.Data[2]] = shortaddr;  //对应的房间设备的路由表建立                    
            #ifdef DEBUG_STAGE 
                     HalUARTWrite(0,&asc_16[Routing_Table[pkt->cmd.Data[1]][pkt->cmd.Data[2]]/4096],1);
                     HalUARTWrite(0,&asc_16[Routing_Table[pkt->cmd.Data[1]][pkt->cmd.Data[2]]%4096/256],1);
                     HalUARTWrite(0,&asc_16[Routing_Table[pkt->cmd.Data[1]][pkt->cmd.Data[2]]%256/16],1);
                     HalUARTWrite(0,&asc_16[Routing_Table[pkt->cmd.Data[1]][pkt->cmd.Data[2]]%16],1);
            #endif           
             buffer[0] = 'r';//进行回复          
             buffer[1] = 0;
             buffer[2] = 0;
             buffer[3] = 'a';
             SampleApp_Point_to_Point_DstAddr.addr.shortAddr = shortaddr;//并把应答信息发送给终端 入网成功
             if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_ADDNET_CLUSTERID,
                           4,
                           buffer,
                           &SampleApp_TransID,
                           AF_DISCV_ROUTE | AF_ACK_REQUEST,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
            {
             //  HalUARTWrite(0,"get searchmsg\n\r",sizeof("get searchmsg\n\r"));      
             //  HalUARTWrite(0,"bind one end",12);
               HalLedSet(LED1,LED_OFF);//协调者led1灯灭 表示曾找到网络
            }
            else
            {
              // Error occurred in request to send.
            }
         }
    #else        //终端处理入网应答
    
     if((pkt->cmd.Data[0]=='r' && pkt->cmd.Data[3]=='a') && pkt->srcAddr.addr.shortAddr == 0x0000)
     {
      #ifdef DEBUG_STAGE
        HalUARTWrite(0,"bind success",12);
      #endif
        HalLedSet(LED1,LED_ON);//找到网络就开启LED1
     }
     else
     {
        #ifdef DEBUG_STAGE
            HalUARTWrite(0,"bind fail",9);
        #endif
        HalLedSet(LED1,LED_OFF);//未找到网络就关闭LED1
     }
     #endif
      break;
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
    /*  flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );*/
      break;
  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  /*
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
  */
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */

void SampleApp_SendFlashMessage( uint16 flashTime )
{
  /*
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
  */
}


/*********************************************************************
 * @fn      SampleApp_SerialCMD
 *
 * @brief   串口给的数据处理
 *
 *
 * @return  none
 */
#ifdef DEV_COOR
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg) 
{
  uint8 dev_num,len,*str = NULL;//len有用的数据长度
  str = (cmdMsg->msg+1);//指向真正的数据
  len = *(cmdMsg->msg);//指向数据头 就是有多少数据
  /****************以下为串口打印接收到的数据*/
  #ifdef DEBUG_STAGE
    int i;
    for(i = 0; i <= len; i++)
    HalUARTWrite(0,str+i,1);
    HalUARTWrite(0,"\n",1);
  #endif
    /******仿照点对点的发送模式***/
    /*
    afAddrType_t SampleApp_Point_to_Point_DstAddr;
    SampleApp_Point_to_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_Point_to_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    ampleApp_Point_to_Point_DstAddr.addr.shortAddr = 0xffff;//数据从终端发给协调者
    */
    if(str[CmdFlag] == FRAME_FLAG && len == CMD_LEN)//判断框架有没有正确 长度有没有正确
    {
      for(dev_num = 1; dev_num <= DEV_COUNT; dev_num++)
      {
        if(Routing_Table[str[CmdRoomId]][dev_num] == 0)
        {
           str[CmdRoomId] = ERROR_FLAG;
           HalUARTWrite(0,str,CMD_LEN);
           return;
        }
       SampleApp_Point_to_Point_DstAddr.addr.shortAddr = Routing_Table[str[CmdRoomId]][dev_num];
             if ( AF_DataRequest( &SampleApp_Point_to_Point_DstAddr,
                       &SampleApp_epDesc,
                       SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                       len,//数据长度
                       str,//数据内容
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                {
                  }
             else
             {
                // Error occurred in request to send.
             }
     }
 }
}
#endif