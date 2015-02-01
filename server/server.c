#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <syslog.h>
#include <pthread.h>
#include <error.h>
#include "package.h"

#define MAX_CLIENT_COUET 50 // the max number of client is 50
typedef struct{
  int sock_fd;
  char ip_addr[20];
  pclientlink new_node;
}client_info;
/***funtion declare***/
void periodUpdata();
void tcpService(void *ir);
void showCmd( CmdInfo *msg);
/**************************/
/********global value declare************/
pthread_mutex_t mutex;
/************************/
static int client_count; //global static value

int uart_fd;
CmdInfo *global_uart_cmd_send;//uart send cmd
CmdInfo *global_uart_cmd_recv;//uart recv cmd
unsigned char *uart_recv_buf;//recv buffer
/**********CMD ROOM DEV VALUE*************/
LIVINGROOM_DevValue   *Lr_De_Va;
BEDROOM_DevValue      *Br_De_Va;
KITCHENROOM_DevValue  *Kr_De_Va;
// char client_ip_array[MAX_CLIENT_COUNT][IP_LEN] = {0};
pclientlink head;
int time_count = 0;


int main(int argc, char *argv[] )
{
//clientlist Init
  ClientLink_Init(&head);
  /**************init mutex***************/
  if(pthread_mutex_init(&mutex, NULL) < 0)
  { 
    printf("fail to mutex_init\n");
    exit(-1);
  }
  //global value init 
  global_uart_cmd_send = (CmdInfo*)malloc(CMDINFO_LEN);
  global_uart_cmd_recv = (CmdInfo*)malloc(CMDINFO_LEN);
  uart_recv_buf = (unsigned char*)malloc(CMDINFO_LEN);//malloc uart data
  Lr_De_Va = (LIVINGROOM_DevValue*)malloc(sizeof(LIVINGROOM_DevValue));
  Br_De_Va = (BEDROOM_DevValue*)malloc(sizeof(BEDROOM_DevValue));
  Kr_De_Va = (KITCHENROOM_DevValue*)malloc(sizeof(KITCHENROOM_DevValue));
  client_count = 0;//init client count
  //init uart
  uart_fd = UART_Open(UART1);//uart open in serial.h
  UART_Init(uart_fd);//uart init
  // init timer for period updata'
  struct itimerval value;
  value.it_value.tv_sec =         10;
  value.it_value.tv_usec =         0;
  value.it_interval.tv_sec =      10;
  value.it_interval.tv_usec =      0;
  signal(SIGALRM,periodUpdata);//get timer signal
  //printf("waiting\n");
  periodUpdata();//client pthread task
  setitimer(ITIMER_REAL,&value,NULL);
  //socket init
  pthread_attr_t p_pth_attr;
  if(pthread_attr_init(&p_pth_attr) != 0)
  {
     perror("pthread_attr_init");
     return -1;
  }
  if(pthread_attr_setdetachstate(&p_pth_attr,PTHREAD_CREATE_DETACHED))
  {
     perror("pthread_attr_setdetachstate");
     return -1;
  }
  
  int i,ip_exist;
  fd_set rdfs;
  int serv_sock,new_sock;

  pthread_t server_th,pthread_t_ret;
  int sin_size;
  client_info sample_client_info;
  unsigned short serverport = 8888;
  struct sockaddr_in their_addr;
  sin_size = sizeof(struct sockaddr_in);
  serv_sock = open_sock(serverport);

  printf("Waiting for connection\n");
  while(1)
  {
    ip_exist = 0;
    FD_ZERO(&rdfs);
    FD_SET(serv_sock,&rdfs);
    if(select(serv_sock+1,&rdfs,NULL,NULL,NULL) < 0)
       {
         printf("fail to select");
	 exit(-1);
       }
    if(FD_ISSET(serv_sock,&rdfs))
    {
       if((new_sock = accept(serv_sock,(struct sockaddr*)&their_addr,&sin_size)) == -1)
       {
         continue;
       }

       if(client_count < MAX_CLIENT_COUET)
       {
         printf("get connection from %s",inet_ntoa(their_addr.sin_addr));
	 pclientlink new_client_node = (pclientlink)malloc(sizeof(clientlink));
	 new_client_node->connected_flag = 0;
	 strcpy(new_client_node->ip_addr , inet_ntoa(their_addr.sin_addr));
	 new_client_node->sock_fd = new_sock;

	 sample_client_info.new_node = new_client_node;
	 sample_client_info.sock_fd  = new_sock;
	 strcpy( sample_client_info.ip_addr,inet_ntoa(their_addr.sin_addr));
	 if(pthread_create(&server_th,&p_pth_attr,(void *)tcpService,&sample_client_info) == 0)
	 {
	   new_client_node->client_pthread_t = server_th;
	   ClientLink_Insert(head,new_client_node);
	   client_count++;
	   printf("newpthread_t = %ld,newip:%s,client_count = %d\n",head->next->client_pthread_t,head->next->ip_addr,client_count);
	 }
	 else
	 {
	   printf("Warning:create pthread error!\n");
	 }
       }
       else
       {
          printf("Waring:reach the client connection upper limit!\n");
       }
    }
  }
  printf("server exit\n");
  free(global_uart_cmd_send);
  free(global_uart_cmd_recv);
  free(uart_recv_buf);
  free(Lr_De_Va);
  free(Br_De_Va);
  free(Kr_De_Va);
  close(serv_sock);
  UART_Close(uart_fd);
  return 0;
}

void msgInit(CmdInfo *msg)
{
  msg->CmdFlag =        1;
  msg->CmdType =        0;
  msg->CmdCtrlType =    0;
  msg->CmdCtrlValue =   0;
  msg->CmdRoomId =      1;
  msg->CmdDevType =     0;
  msg->CmdDevId   =     0;
}

CmdInfo * uart_send_and_recv(CmdInfo * uart_send_temp)
{
  int ret;
  int Timeout_Count = 0;
  //memset(uart_recv_buf,0,CMDINFO_LEN);
  #ifdef DEBUG
   printf("send to uart:\n");
   showCmd(uart_send_temp);
  #endif
  while(1)
  {
    Timeout_Count++;
    memset(uart_recv_buf,0,CMDINFO_LEN);
    pthread_mutex_lock(&mutex);
      UART_Send(uart_fd,(char*)uart_send_temp,UART_SEND_LEN);
      ret = UART_Recv(uart_fd,uart_recv_buf);
    pthread_mutex_unlock(&mutex);
    usleep(20);
    if(ret != -1 && uart_recv_buf[0] == FRAME_FLAG)
    {
     break;
    }
    if(Timeout_Count == 3)
    {
     printf("recv zigbee error \n");
     break;
    }
  }
  #ifdef DEBUG
     printf("recv from uart:\n");
     showCmd((CmdInfo*)uart_recv_buf);
  #endif
    return (CmdInfo*)uart_recv_buf;
}

void periodUpdata()
{
 int i,ret;
 pclientlink p = head;
 
 time_count++;
 if(time_count%3 == 0)
 {
   while(p->next != NULL)
   {
     p = p->next;
     if(p->connected_flag == 0)
     {
        printf("ip : %s time out! kill pthread:%ld\n",p->ip_addr,p->client_pthread_t);
	      pthread_cancel(p->client_pthread_t);
	      ClientLink_Remove(head,p);
	      client_count--;
     }
     else
     {
       p->connected_flag = 0;
     }
   }
 }
 
 
   // the same word in all reponse
    global_uart_cmd_send->CmdFlag = FRAME_FLAG;
    global_uart_cmd_send->CmdType = CMD_TYPE_QUERYREQUEST;
    global_uart_cmd_send->CmdCtrlType = CMD_CONTROL_READ;
    global_uart_cmd_send->CmdCtrlValue = 0x6f;
    //global_uart_cmd_send->CmdModeType = 0x0;
	   //global_uart_cmd_send->CmdModeValue = 0x0;
	 //global_uart_cmd_send->CmdPreHValue = 0x0;
	 	
    #ifdef LIVINGROOM
        global_uart_cmd_send->CmdRoomId = CMD_ROOM_LIVINGROOM;
         for(i=0;i<=2;i++ )
	      {
	 	      global_uart_cmd_send->CmdDevId = i;
	 	      global_uart_cmd_send->CmdDevType= LIVINGROOM_DevType_Array[0];	//LIVINGROOM_DevType_Array[0]=CMD_DEV_TYPE_LED
		      global_uart_cmd_recv=uart_send_and_recv(global_uart_cmd_send); //uart send and recv
		      *(&(Lr_De_Va->DEV_LED0_VALUE)+i)= global_uart_cmd_recv->CmdCtrlValue;//add value into local struct  0x6f
	      }
	      global_uart_cmd_send->CmdDevId = 0;
	       for(i=1;i< LIVINGROOM_DevType_Count;i++)
	       {
	 	        global_uart_cmd_send->CmdDevType = LIVINGROOM_DevType_Array[i];	
		        global_uart_cmd_recv=uart_send_and_recv(global_uart_cmd_send); //uart send and recv
		        *(&(Lr_De_Va->DEV_LED0_VALUE)+i)= global_uart_cmd_recv->CmdCtrlValue;//add value into local struct
	       }
    #endif
    #ifdef BEDROOM//BEDROOM Updata
    global_uart_cmd_send->CmdRoomId = CMD_ROOM_BEDROOM;
    global_uart_cmd_send->CmdDevId = 0;
	   for(i = 0;i < BEDROOM_DevType_Count;i++)
	  {
	   	global_uart_cmd_send->CmdDevType = BEDROOM_DevType_Array[i];	
		  global_uart_cmd_recv=uart_send_and_recv(global_uart_cmd_send); //uart send and recv
	   	usleep(1000);
	    //	showCmd(global_uart_cmd_recv);
		 *(&(Br_De_Va->DEV_LED0_VALUE)+i)= global_uart_cmd_recv->CmdCtrlValue;//add value into local struct
	 }
	    #ifdef DEBUG
		     printf("bedroom uart recv:%d %d  %d %d %d %d\n",Br_De_Va->DEV_LED0_VALUE,Br_De_Va->DEV_LED1_VALUE,Br_De_Va->DEV_CURTAIN_VALUE,\
		     Br_De_Va->DEV_WINDOWMAGNETIC_VALUE,Br_De_Va->DEV_TEMPERATURE_VALUE,Br_De_Va->DEV_LIGHTINTENSITY_VALUE);
     #endif	 
   #endif
   #ifdef KITCHENROOM//KITCHENROOM Updata
	 global_uart_cmd_send->CmdRoomId = CMD_ROOM_KITCHEN;
	 global_uart_cmd_send->CmdDevId = 0;
	 for(i = 0;i < KITCHENROOM_DevType_Count;i++)
	 {
	 	global_uart_cmd_send->CmdDevType = KITCHENROOM_DevType_Array[i];	
		printf("uart_send:\n");
		showCmd(global_uart_cmd_send);
		global_uart_cmd_recv=uart_send_and_recv(global_uart_cmd_send); //uart send and recv
		printf("uart_recv:\n");
		showCmd(global_uart_cmd_recv);
		*(&(Kr_De_Va->DEV_LED0_VALUE)+i)= global_uart_cmd_recv->CmdCtrlValue;//add value into local struct
	 }	
   #endif//KITCHENROOM Updata(end)


   #ifdef DEBUG 
       printf("\n------------------------------------------\n");
   #ifdef LIVINGROOM
	       printf("livingroom local vlaue:");
	      for(i=0;i<=LIVINGROOM_DevType_Count;i++)
		       printf("%d  ",*(&(Lr_De_Va->DEV_LED0_VALUE)+i));
	      printf("\n");
    #endif
   #ifdef BEDROOM
	     printf("bedroom local vlaue:");
	     for(i=0;i<BEDROOM_DevType_Count;i++)
		     printf("%d  ",*(&(Br_De_Va->DEV_LED0_VALUE)+i));
	       printf("\n");
   #endif
   #ifdef KITCHENROOM
	    printf("kitchenroom local vlaue:");
	     for(i=0;i<KITCHENROOM_DevType_Count;i++)
		    printf("%d  ",*(&(Kr_De_Va->DEV_LED0_VALUE)+i));
	    printf("\n");
   #endif
#endif
//show client list
#ifdef DEBUG
	p=head;
	i=1;
	printf("client list:\n");
	while(p->next != NULL)
	{
		p = p->next;
		printf("client%d:%s\n",i,p->ip_addr);
		i++;
	}
#endif
}

void tcpService(void *ir)
{
  fd_set this_rdfs;
  client_info this_client_info;
  this_client_info.sock_fd = (*(client_info*)ir).sock_fd;
  strcpy(this_client_info.ip_addr,(*(client_info*)ir).ip_addr);
  this_client_info.new_node = (*(client_info*)ir).new_node;

  int ret,select_ret,i;
  pclientlink p =  head;
  CmdInfo *tcp_cmd_recv;
  CmdInfo *tcp_cmd_send;
  CmdInfo *uart_cmd_send;
  tcp_cmd_recv = (CmdInfo*)malloc(CMDINFO_LEN);
  tcp_cmd_send = (CmdInfo*)malloc(CMDINFO_LEN);
  uart_cmd_send = (CmdInfo*)malloc(CMDINFO_LEN);
  for(;;)
  {
    memset(tcp_cmd_recv,0,CMDINFO_LEN);
    memset(tcp_cmd_send,0,CMDINFO_LEN);
    FD_ZERO(&this_rdfs);
    FD_SET(this_client_info.sock_fd,&this_rdfs);
    select_ret = select(this_client_info.sock_fd+1,&this_rdfs,NULL,NULL,NULL);
    if(select_ret <= 0)
    {
       if(errno == EINTR)
       continue;
       else
       printf("tcp select error\n");
    }
    if(!FD_ISSET(this_client_info.sock_fd,&this_rdfs))
    {
       continue;
    }
    ret = read(this_client_info.sock_fd,(unsigned char*)tcp_cmd_recv,CMDINFO_LEN);
    if(ret <= 0)
    {
      printf("One Client exit\n");
      break;
    }
    if(ret != CMDINFO_LEN)
    continue;
#ifdef DEBUG
  printf("ret = %d,recv from client:%s\n",ret,this_client_info.ip_addr);
  showCmd(tcp_cmd_recv);
#endif
  this_client_info.new_node->connected_flag = 1;
    if(tcp_cmd_recv->CmdFlag != FRAME_FLAG)
     continue;
    if(tcp_cmd_recv->CmdDevType == FRAME_FLAG)
     {
        tcp_cmd_send->CmdFlag = FRAME_FLAG;
	      tcp_cmd_send->CmdType = CMD_TYPE_QUERYREQUEST;
	      tcp_cmd_send->CmdCtrlType = CMD_CONTROL_READ;
	      tcp_cmd_send->CmdCtrlValue = 0x6f;
	      
	#ifdef LIVINGROOM//LIVINGROOM Updata
		tcp_cmd_send->CmdRoomId = CMD_ROOM_LIVINGROOM;//roomid
		// 2 led in livingroom		
	 	tcp_cmd_send->CmdDevId = 0;//led0
	 	tcp_cmd_send->CmdDevType = LIVINGROOM_DevType_Array[0];//led表示的是LED
		tcp_cmd_send->CmdCtrlValue = Lr_De_Va->DEV_LED0_VALUE;	//get value from local struct
		ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;//send to socket
		if(ret < 0)
			printf("socket send error\n");
    #ifdef DEBUG
		 printf("send to client:\n");
		 showCmd(tcp_cmd_send);
     #endif
 
		tcp_cmd_send->CmdDevId = 1;//led1
		tcp_cmd_send->CmdCtrlValue = Lr_De_Va->DEV_LED1_VALUE;//get value from local struct
		ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;	//send to socket
		if(ret < 0)
			printf("socket send error\n");
#ifdef DEBUG
	      printf("send to client:\n");
	      showCmd(tcp_cmd_send);
#endif

		tcp_cmd_send->CmdDevId = 1;//led1
		tcp_cmd_send->CmdCtrlValue = Lr_De_Va->DEV_LED2_VALUE;//get value from local struct
		ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;	//send to socket
		if(ret < 0)
			printf("socket send error\n");
#ifdef DEBUG
	      printf("send to client:\n");
	      showCmd(tcp_cmd_send);
#endif
 		// 3 led in livingroom(end)
		//remain senior in living room
 		tcp_cmd_send->CmdDevId = 0;//remain  devid is all 0
 		for(i=1;i<LIVINGROOM_DevType_Count;i++)
 		{
 			tcp_cmd_send->CmdDevType = LIVINGROOM_DevType_Array[i];//get devtype from array in order
 			tcp_cmd_send->CmdCtrlValue = *(&(Lr_De_Va->DEV_LED0_VALUE)+i);//get value from local struct in order
			ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;//send to socket
			if(ret < 0)
				printf("socket send error\n");
#ifdef DEBUG
			printf("send to client:\n");
		  showCmd(tcp_cmd_send);
#endif
 		}
		//remain senior in living room(end)			
#endif//LIVINGROOM Updata	(end)


#ifdef BEDROOM//BEDROOM Updata 
		 tcp_cmd_send->CmdRoomId = CMD_ROOM_BEDROOM;//roomid
		 tcp_cmd_send->CmdDevId = 0;//devid is all 0
		 for(i=0;i<BEDROOM_DevType_Count;i++)
		{
			tcp_cmd_send->CmdDevType = BEDROOM_DevType_Array[i];//get devtype from array in order
 			tcp_cmd_send->CmdCtrlValue = *(&(Br_De_Va->DEV_LED0_VALUE)+i);//get value from local struct in order
			ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;	//send to socket
		 	if(ret < 0)
				printf("socket send error\n");
     #ifdef DEBUG
			    printf("send to pc:\n");
		     showCmd(tcp_cmd_send);
     #endif
		}
#endif//BEDROOM Updata (end)


#ifdef KITCHENROOM//KITCHENROOM Updata
		 tcp_cmd_send->CmdRoomId = CMD_ROOM_KITCHEN ;//roomid
		 tcp_cmd_send->CmdDevId = 0;//devid is all 0
		 for(i=0;i<KITCHENROOM_DevType_Count;i++)
		{
			tcp_cmd_send->CmdDevType = KITCHENROOM_DevType_Array[i];//get devtype from array in order
 			tcp_cmd_send->CmdCtrlValue = *(&(Kr_De_Va->DEV_LED0_VALUE)+i);//get value from local struct in order
			ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;//send to socket	
#ifdef DEBUG
	   printf("send to pc:\n");
	   showCmd(tcp_cmd_send);
#endif
		}
#endif//KITCHENROOM Updata(end)


	continue;
     }
   /*  
      else if(tcp_cmd_recv->CmdModeType != CMD_MODE_TYPE_SERCUITY)
	  {
	  	switch(tcp_cmd_recv->CmdModeType)
	  	{
			case CMD_MODE_TYPE_SLEEP:
				 uart_cmd_send->CmdFlag = FRAME_FLAG;//flag
				 uart_cmd_send->CmdType = CMD_TYPE_CTRL_REQUEST;  
				 uart_cmd_send->CmdCtrlType = CMD_CTRL_TYPE_CLOSE;
				 uart_cmd_send->CmdCtrlValue = 0x6f;//value
				 uart_cmd_send->CmdDevType = CMD_DEV_TYPE_LED;
				 #ifdef LIVINGROOM
				 uart_cmd_send->CmdRoomId = CMD_ROOM_LIVING_ROOM;
				 uart_cmd_send->CmdDevId = 0;
				 tcp_cmd_send=uart_send_and_recv(uart_cmd_send);//uart send and recv
				 uart_cmd_send->CmdDevId = 1;
				 tcp_cmd_send=uart_send_and_recv(uart_cmd_send);//uart send and recv
        #endif
        #ifdef BEDROOM
				uart_cmd_send->CmdRoomId = CMD_ROOM_BED_ROOM;
				 uart_cmd_send->CmdDevId = 0;
				 tcp_cmd_send=uart_send_and_recv(uart_cmd_send);//uart send and recv
        #endif
        #ifdef KITCHENROOM
				uart_cmd_send->CmdRoomId = CMD_ROOM_KITCHEN_ROOM;
				 uart_cmd_send->CmdDevId = 0;
				 tcp_cmd_send=uart_send_and_recv(uart_cmd_send);//uart send and recv
        #endif
				tcp_cmd_send->CmdModeType = CMD_MODE_TYPE_SLEEP;
				tcp_cmd_send->CmdCtrlValue = 1;
				ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;	//send to socket				 
				break;
			case CMD_MODE_TYPE_WAKEUP:
      #ifdef LIVINGROOM
				uart_cmd_send->CmdFlag = FRAME_FLAG;//flag
				uart_cmd_send->CmdType = CMD_TYPE_CTRL_REQUEST;  
				uart_cmd_send->CmdCtrlType = CMD_CTRL_TYPE_OPEN;
				uart_cmd_send->CmdCtrlValue = 0x6f;//value
				uart_cmd_send->CmdDevType = CMD_DEV_TYPE_CURTAIN;
				uart_cmd_send->CmdRoomId = CMD_ROOM_LIVING_ROOM;
				uart_cmd_send->CmdDevId = 0;
				tcp_cmd_send=uart_send_and_recv(uart_cmd_send);//uart send and recv
				tcp_cmd_send->CmdModeType = CMD_MODE_TYPE_WAKEUP;
				tcp_cmd_send->CmdCtrlValue = 1;
				ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;	//send to socket	
       #endif
				break;
			case CMD_MODE_TYPE_OUTHOME:
				break;
		}
  }*/
  
  
  /*********************signal control & updata********************/
	  else if(tcp_cmd_recv->CmdType == CMD_TYPE_QUERYREQUEST  ||tcp_cmd_recv->CmdType == CMD_TYPE_CONTROLREQUEST)
	  {
	  	if(tcp_cmd_recv->CmdCtrlType == CMD_CONTROL_OPEN  || tcp_cmd_recv->CmdCtrlType == CMD_CONTROL_CLOSE\
														||tcp_cmd_recv->CmdCtrlType == CMD_CONTROL_READ )
	  	{
	  		if(tcp_cmd_recv->CmdRoomId == CMD_ROOM_BEDROOM  || tcp_cmd_recv->CmdRoomId == CMD_ROOM_KITCHEN\
															   || tcp_cmd_recv->CmdRoomId ==  CMD_ROOM_LIVINGROOM)
	  		{
			tcp_cmd_send = uart_send_and_recv(tcp_cmd_recv);//uart send and recv
			ret = write_sock(this_client_info.sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;	//send to socket	
			//updata change data  to local struct
			switch(tcp_cmd_send->CmdRoomId )
			{	
#ifdef LIVINGROOM
			case  CMD_ROOM_LIVINGROOM://livingroom
				switch(tcp_cmd_send->CmdDevType)
				{
					case CMD_DEV_LED:
						if(tcp_cmd_send->CmdDevId == 0)
							{Lr_De_Va->DEV_LED0_VALUE = tcp_cmd_send->CmdCtrlValue;}
						else if(tcp_cmd_send->CmdDevId == 1)
							{Lr_De_Va->DEV_LED1_VALUE = tcp_cmd_send->CmdCtrlValue;}
							else
								{Lr_De_Va->DEV_LED2_VALUE = tcp_cmd_send->CmdCtrlValue;}
						break;
					case CMD_DEV_CURTAIN:
						Lr_De_Va->DEV_CURTAIN_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
					case CMD_DEV_DOORBELL:
						Lr_De_Va->DEV_DOORBELL_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
					
					case CMD_DEV_WINDOWMAGNETIC:
						Lr_De_Va->DEV_WINDOWMAGNETIC_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
				}
				break;//livingroom break
#endif
#ifdef BEDROOM
			case CMD_ROOM_BEDROOM://bedroom
				switch(tcp_cmd_send->CmdDevType)
				{
					case CMD_DEV_LED:	
						if(tcp_cmd_send->CmdDevId == 0)
							{Br_De_Va->DEV_LED0_VALUE = tcp_cmd_send->CmdCtrlValue;}
							else
						  {Br_De_Va->DEV_LED1_VALUE = tcp_cmd_send->CmdCtrlValue;}
						break;
				case CMD_DEV_CURTAIN:
						Br_De_Va->DEV_CURTAIN_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
				case CMD_DEV_WINDOWMAGNETIC:
						Br_De_Va->DEV_WINDOWMAGNETIC_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
					case CMD_DEV_TEMPERATURE:
						Br_De_Va->DEV_TEMPERATURE_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
					case  CMD_DEV_LIGHTINTENSITY:
						Br_De_Va->DEV_LIGHTINTENSITY_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
				}
				break;//bedroom break
#endif
#ifdef KITCHENROOM
			case CMD_ROOM_KITCHEN://kitchenroom
				switch(tcp_cmd_send->CmdDevType)
				{
					case CMD_DEV_LED :
						Kr_De_Va->DEV_LED0_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
					case CMD_DEV_AIR:
						Kr_De_Va->DEV_AIR_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;
					case  CMD_DEV_WINDOWMAGNETIC:
						Kr_De_Va->DEV_WINDOWMAGNETIC_VALUE = tcp_cmd_send->CmdCtrlValue;
						break;	
				}
				break;//kitchenroom break	
#endif
			}
					
#ifdef DEBUG
	   	printf("control cmd send to client:\n");
      showCmd(tcp_cmd_send);
#endif
			if(tcp_cmd_send->CmdType ==CMD_TYPE_CONTROLRESPONSE)
			{
				 p=head;
#ifdef DEBUG
				printf("\n\nstatus change !!%s send to follow clients :\n",this_client_info.ip_addr);
#endif
				while(p->next != NULL)//广播状态改变
				{
					p = p->next;
					if(p->sock_fd == this_client_info.sock_fd)
						continue;
					
					ret = write_sock(p->sock_fd, (unsigned char *)tcp_cmd_send, CMDINFO_LEN) ;	//send to socket
					printf("                      ip=%s\n",p->ip_addr);				
				}
				 showCmd(tcp_cmd_send);
		  		}		
	  		}
		}
	  }
	  /*********************signal control & updata(end)********************/

 }
 	ClientLink_Remove(head, this_client_info.new_node);
	printf("%s exit\n",this_client_info.ip_addr);
	client_count--;
	printf("client_count=%d\n",client_count);
         close_sock(this_client_info.sock_fd);
 	 pthread_exit(NULL);

}

void showCmd(CmdInfo *msg)
{
  #ifdef DEBUG
	  printf("%d   ", msg->CmdFlag) ;           /* 命令类型 */
	  printf("Type:%d   ", msg->CmdType) ;           /* 命令类型 */
      printf("CtrlType:%d   ", msg->CmdCtrlType);    /* 命令控制的类型 */
      printf("Value:%d    ", msg->CmdCtrlValue);  /* 命令值 */
      printf("RoomId:%d   ", msg->CmdRoomId);        /* 房间号 */
      printf("DevType:%d   ", msg->CmdDevType);      /* 设备类型 */
     printf("DevId:%d    ", msg->CmdDevId);          /* 设备号 */  
      //printf("%d ", msg->CmdModeType);          /* 设备号 */  
      //printf("%d ", msg->CmdModeValue);          /* 设备号 */  
      //printf("%d ", msg->CmdPreHValue);          /* 设备号 */
      //printf("%d\n\n", msg->CmdPreLValue);          /* 设备号 */  
	#endif
}
