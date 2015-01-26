#termialv2的使用
该版本的termial的基本上是直接可以在硬件上跑起来了，而且他是有相应的IO口配置，将能用到的IO基本都已经使用起来了
##IO口的配置
在这个配置中，您是可以定制自己的使用的设备，当你想使用什么设备，只要在编译过程中将相应的宏定义加入即可！

房间定义

* LIVING_ROOM         客厅
* BED_ROOM            卧室
* KITCHEN_ROOM        厨房

设备定义

* DEV_LIGHT          灯   **P0.0**  输出 **P0.1**  输出 **p0.4** 输出
* DEV_CURTAIN       窗帘      **P0.5**   输出
* DEV_TEMPERATURE    温度     **P0.6**   输入
* DEV_LIGHT         光强        **P0.7**  
* DEV_IAR          烟雾         **P1.2**   输入
* DEV_WINDOWMAGNETIC     窗磁     **P1.5**   输入
* DEV_DOORBELL        门铃      **P2.0**   中断 
* DEV_OUTPUT         输出预留     **P1.6 P1.7**
* DEV_INPUT          输入预留       **p2.3 p2.4**

##最不要使用的IO口
LED1    联网状态   P1.0  找到为1  未找到为0  一直亮

LED2   命令得到状态  P1.1 得到为1发动出去为0  一直一会亮一会不亮

P0.2 P0.3不能用 为串口数据格式

##CMD命令定义
	#define  CmdFlag           0           /*帧头*/             7f                       
	#define  CmdType           1          /* 命令类型 */        00                           
	#define  CmdCtrlType       2         /* 命令控制的类型 */    00
	#define  CmdCtrlValue      3        /* 命令值 */             00                    
	#define  CmdRoomId         4        /* 房间号 */             00                   
	#define  CmdDevType        5        /* 设备类型 */           00                 
	#define  CmdDevId          6        /* 设备号 */             00                     
##查询表单如下：
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
  
##可得的命令模式
定义的数目模型为 7f 0X 0X 0X 0X 0X 0X


