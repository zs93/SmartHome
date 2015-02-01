#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <sys/types.h>//open() head file
#include <sys/stat.h>
#include <fcntl.h>
 #include <termios.h>//set uart head file
 #include <unistd.h>
#include <string.h>


#define UART1 "/dev/ttySAC1" //arm dev con2
#define BUF_SIZE 128 //serial can get data 128 bit

int UART_Open(char *port);
void UART_Init(int fd);
void UART_Close(int fd);
int UART_Send(int fd,char *send_buf,int data_len);
int UART_Recv(int fd,char *rcv_buf);
#endif

