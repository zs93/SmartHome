#ifndef CMD_H
#define CMD_H
/*****CmdType*****/
#define CMD_TYPE_CLOSELINK                 0x00 // close link
#define CMD_TYPE_QUERYREQUEST              0x01 //query request
#define CMD_TYPE_QUERYSPONSE               0X02 //query sponse
#define CMD_TYPE_CONTROLREQUEST            0x03 //control request
#define CMD_TYPE_CONTROLRESPONSE           0x04  //control response
/******CmdCtrlType***********/
#define CMD_CONTROL_OPEN                   0x00  //open
#define CMD_CONTROL_CLOSE                  0x01  //close
#define CMD_CONTROL_WRITE                  0x02  //write
#define CMD_CONTROL_READ                   0x03  //read
#define CMD_CONTROL_START                  0x04  //start
#define CMD_CONTROL_STOP                   0x05  //stop
/********CmdRoomId*******/
#define CMD_ROOM_LIVINGROOM                0x00  //livingroom
#define CMD_ROOM_BEDROOM                   0x01  //bedroom
#define CMD_ROOM_KITCHEN                   0x02  //kitchen
/**********CmdDevType**************/
#define CMD_DEV_LED                        0x00  //light
#define CMD_DEV_CURTAIN                    0x01  //curtain
#define CMD_DEV_TEMPERATURE                0x02  //temperture
#define CMD_DEV_LIGHTINTENSITY             0x03  //light intensity
#define CMD_DEV_AIR                        0x04  //AIR
#define CMD_DEV_WINDOWMAGNETIC             0x05  //window magnetic
#define CMD_DEV_DOORBELL                   0x06  //doorbell
#define CMD_DEV_OUTPUT                     0x07  //OUTPUT
#define CMD_DEV_INPUT                      0x08  //input
/*********************/
#define CMD_DEV_ALL                        0xff  //all dev

/***********cmd mode type************/
/*
#define CMD_MODE_TYPE_SERCUITY     0x00 //sercuity mode
#define CMD_MODE_TYPE_SLEEP        0x01 //sleep mode
#define CMD_MODE_WAKEUP            0x02 //wakeup mode
#define CMD_MODE_OUTHOME           0x03 //out home mode
*/
/***********cmd mode value *******/
/*
#define CMD_MODE_WARNINGCLOSE     0x01  //close waring
#define CMD_MODE_ALLLEDOFF        0x02  //all led close
*/
#pragma pack(1) //start align at one byte
typedef struct
{
  unsigned char CmdFlag;     //flag
  unsigned char CmdType;     //cmd  type 
  unsigned char CmdCtrlType;    //control type
  unsigned char CmdCtrlValue;  //control value
  unsigned char CmdRoomId;     // room id
  unsigned char CmdDevType;   //device type 
  unsigned char CmdDevId;     //device id
//  unsigned char CmdModeType;  //mode type
//  unsigned char CmdModeValue; // mode value
//  unsigned char CmdPreHValue;
//  unsigned char CmdPreLValue;
}CmdInfo;
#pragma pack() //stop align at one byte
/******living room value*****/

typedef struct
{
  unsigned char DEV_LED0_VALUE;
  unsigned char DEV_LED1_VALUE;
  unsigned char DEV_LED2_VALUE;
  unsigned char DEV_CURTAIN_VALUE;
  unsigned char DEV_WINDOWMAGNETIC_VALUE;
  unsigned char DEV_DOORBELL_VALUE;
}LIVINGROOM_DevValue;
/************************************/

/**********bed room value********/
typedef struct
{
  unsigned char DEV_LED0_VALUE;
  unsigned char DEV_LED1_VALUE;
  unsigned char DEV_CURTAIN_VALUE;
  unsigned char DEV_WINDOWMAGNETIC_VALUE;
  unsigned char DEV_TEMPERATURE_VALUE;
  unsigned char DEV_LIGHTINTENSITY_VALUE;
}BEDROOM_DevValue;
/************************************/
/*********kitchen value*************/
typedef struct
{
  unsigned char DEV_LED0_VALUE;
  unsigned char DEV_AIR_VALUE;
  unsigned char DEV_WINDOWMAGNETIC_VALUE;
}KITCHENROOM_DevValue;
/****************************************/
#define LIVINGROOM_DevType_Count   4
#define BEDROOM_DevType_Count      5
#define KITCHENROOM_DevType_Count  3

unsigned char LIVINGROOM_DevType_Array[LIVINGROOM_DevType_Count] = {CMD_DEV_LED,\
                                                CMD_DEV_CURTAIN,\
						CMD_DEV_WINDOWMAGNETIC,\
						CMD_DEV_DOORBELL};
unsigned char BEDROOM_DevType_Array[BEDROOM_DevType_Count] = {CMD_DEV_LED,\
                                                  CMD_DEV_CURTAIN,\
						  CMD_DEV_WINDOWMAGNETIC,\
						  CMD_DEV_TEMPERATURE,\
						  CMD_DEV_LIGHTINTENSITY};
unsigned char KITCHENROOM_DevType_Array[KITCHENROOM_DevType_Count] = {CMD_DEV_LED,\
                                                 CMD_DEV_AIR,\
						 CMD_DEV_WINDOWMAGNETIC};

#define CMDINFO_LEN    7
#define UART_SEND_LEN  7
#define ROOM_COUNT     4
#define DEBUG
#define FRAME_FLAG    (0x7f)
/*****************/
#define LIVINGROOM
#define BEDROOM
#define KITCHENROOM
#endif
