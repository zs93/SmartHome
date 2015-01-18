#terminal的使用方法
其实在代码中我已经写好了如何使用，当然在这我还是继续要提供相应的相应的方法来为大家铺路，当然这只是version 1.0，我只实现了一个点灯功能，但是可以在我现在写的协议栈中自己来添加各种相应的代码来实现相应的功能！
##关于自动组网的问题
我其实已经解决了自动联网问题，在此我表示，这是我智慧的结晶，我按照自己的思路想的，用的是一个数组来实现路由绑定的，因此如果您采用了我的带码，那么请您follower我一下，算是对我的开源项目的一点点支持啦！
##关于如何下载使用
这里要说明一下，我的代码全部都是用IAR FOR 8051写的，而且版本为8.10版，因为高级版好像在编译的过程中会出现错误，所以就使用低版本的，如果有人能够使用高版本的IAR来编译的话，希望您能将发送邮件给我，写明一个方法，让我也体验一把高级版本的乐趣。

前面是我废话。来正题了！
###编译协调者：

	/************编译协调者的时候需要将注释去掉**********/
	//#define DEV_COOR
	//#define DEBUG_STAGE
	/*****************************/
>如果您的IAR正常的话，那么在PROJECT - OPTIONS - C/C++ COMPILE - PREPROCESSOR - DEFINED SYMBOLS 添加上面的内容也是行的，主要是我的IAR不太正常，我也不清楚是什么原因造成的！

###编译终端：
在编译终端的情况下，跟上面的一样定义好defined symbols就可以了

	NWK_AUTO_POLL
	ZTOOL_P1
	xMT_TASK
	xMT_SYS_FUNC
	xMT_ZDO_FUNC
	xLCD_SUPPORTED=DEBUG
	LIVING_ROOM
	DEBUG_STAGE
	xDEV_CORE
	DEV_LIGHT
##关于添加相应的功能
其实在sampleapp.c的注释中已经说的非常明白了，在这里我继续重复一遍！
###第一步：设置相应的动作功能为写后面的代码做好准备，就是相应的动作
	
	#define LED1    HAL_LED_1  //自带的LED
	#define LED2    HAL_LED_2
	#define LED3    HAL_LED_3 //不要使用
	#define LED_ON  HAL_LED_MODE_ON//输出为高电平
	#define LED_OFF HAL_LED_MODE_OFF
	/*********************各设备功能模块定义************************
	第一步：在这里添加外设的信息并设置相应的动作*/
	/*****************************************************************
    功能实现：DEV_LIGHT     灯      P0.4   输出
              DEV_CURTAIN   窗帘    P0.0   输出
	***************************************************************/
	//灯 P0.4 输出
	#ifdef  DEV_LIGHT
 	 #define DEV_LIGHT1_OPEN()        do{P0_4 = 1;}while(0);
	  #define DEV_LIGHT1_CLOSE()       do{P0_4 = 0;}while(0);
 	 #define DEV_LIGHT1_BIT           (P0_4)
	#endif

###第二步：终端设备各模块io初始化
找到void SampleApp_Init( uint8 task_id )函数：

	 	/******************终端设备各模块io初始化*******************/
  		/*
                     第二步：这里添加需要的设置的IO初始化
  		***********************************************************/
		#ifndef DEV_COOR
   		 //灯 P0.4 输出
  		#ifdef  DEV_LIGHT
          P0SEL &= 0xef;//灯1配置I/O口  P0_4
          P0DIR |= 0x10;//LIGHT1 输出
          P0_4 = 0;
 	    #endif
 	     //窗帘 P0.0 输出  
 	    #ifdef DEV_CURTAIN
     	  P0SEL &= 0xfe;
     	  P0DIR |= 0x1;  //输出  
     	  P0_0 = 0;
 	   #endif
	   #endif
	   /************IO初始化结束**********************/

###第三步：添加自己的状态机
就是发过来的命令如何处理！找到void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )//接受数据处理函数  就可以写自己相应的代码了！

 	switch ( pkt->clusterId )
 	 {
   	 /***自己添加的条件分支***/
   	 /*****
   	 第三步：添加自己的状态机
   	 ******/
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
      #ifndef DEV_COOR                         
        // HalLedSet(LED2,LED_ON);
          #ifdef DEBUG_STAGE
             for(i=0;i<CMD_LEN;i++)
             HalUARTWrite(0,str+i,1 ); 
          #endif 
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
            
                       }
                   else
                    {
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
            
                       }
                   else
                    {
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
##总结
有关于这个项目的命令数据格式：

	/**命令的类型**/
	#define  CmdFlag         0                             /*帧头	*/                        //0x7f
	#define  CmdType         1                             /* 命令类型 */
	#define  CmdCtrlType     2                             /* 命令控制的类型 */
	#define  CmdCtrlValue    3                             /* 命令值 */
	#define  CmdRoomId       4                             /* 房间号 */
	#define  CmdDevType      5                             /* 设备类型 */
	#define  CmdDevId        6                             /* 设备号 */

>定义的数目模型为 7f 0X 0X 0X 0X 0X 0X
>如果返回值中有6F的话，说明存在错误信息
    