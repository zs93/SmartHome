#include "serial.h"
int UART_Open(char *port)
{
   int fd;
   fd = open(port, O_RDWR|O_NOCTTY|O_NDELAY);
   return fd;
}
void UART_Init(int fd)
{
 struct termios newtio,oldtio;//struct termios please read README.md
 tcgetattr(fd,&oldtio);//get termail fd init value, set this value to oldtio
 cfsetispeed(&newtio,B115200);//set newtio save 115200 speed is input speed
 cfsetospeed(&newtio,B115200);//set newtio save 115200 speed is output speed
 memset(&newtio,0,sizeof(newtio));//set struct newtio all value to be zero
 newtio.c_cflag = CS8 | CREAD ;//get 8 bit and open mode of input
 newtio.c_cflag &= ~ PARENB;
 newtio.c_cflag &= ~ CSTOPB;//set odd number bit and verify bit
 newtio.c_cc[VMIN] = 1;//C_CC read one bit
 newtio.c_cc[VTIME] = 0;
 tcflush(fd,TCIOFLUSH);
 tcsetattr(fd,TCSANOW,&newtio);
}

void UART_Close(int fd)
{
  close(fd);
}

int UART_Send(int fd,char *send_buf,int data_len)
{
  int ret;
  ret = write(fd,send_buf,data_len);
  if(data_len == ret)
  {
    return ret;
  }
  else
  {
   tcflush(fd,TCOFLUSH);
   return -1;
  }
  tcflush(fd,TCOFLUSH);
}

int UART_Recv(int fd,char *rcv_buf)
{
  int len,fs_sel;
  fd_set fs_read;

  struct timeval time;

  FD_ZERO(&fs_read);
  FD_SET(fd,&fs_read);

  time.tv_sec = 2;
  time.tv_usec = 0;
  fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
  if(FD_ISSET(fd,&fs_read))
  {
     len = read(fd,rcv_buf,BUF_SIZE);
     tcflush(fd,TCOFLUSH);
     return len;
  }
  else
  {
     return -1;
  }
}
