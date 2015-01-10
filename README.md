# SmartHome
based on CC2530 and Mini2440
#demo的使用
##串口修改
demo是基于TI提供的cc2530-z-stack2007版本修改
demo中主要实现的是去掉lcd及说明修改串口和组网时的网段的修改

  #if !defined( MT_UART_DEFAULT_OVERFLOW )
  #define MT_UART_DEFAULT_OVERFLOW       FALSE//TRUE
  #endif
  #if !defined MT_UART_DEFAULT_BAUDRATE
  #define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_115200//HAL_UART_BR_38400
  #endif

>此段代码在MT_UART.h中，修改了串口的波特率及流控

##网段修改提供

```/* Define the default PAN ID.
 *
 * Setting this to a value other than 0xFFFF causes
 * ZDO_COORD to use this value as its PAN ID and
 * Routers and end devices to join PAN with this ID
 */
-DZDAPP_CONFIG_PAN_ID=0xFFF0//修改此处为组网需要
```

>此段代码在Tools层下的f8wConfig.cfg文件中 将原来的0xFFFF进行了相应的修改，可见呢，zigbee可组网2^16不同网段，资源还是蛮大的！


##LCD如何去掉？
>IAR->PROJECT->OPTION->C/C++PREPROCESS->
```ZTOOL_P1
xMT_TASK
xMT_SYS_FUNC
xMT_ZDO_FUNC
xLCD_SUPPORTED=DEBUG```
