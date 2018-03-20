#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>

#define BAUDRATE B9600
#define SERIAL_DATA_LENGTH 4097

main()
{
  int Send_Res;
  int fd;
  
  char Send_Dev[]="/dev/ttyUSB0";
  unsigned char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
  
  Send_Buf[0] = 0x02; // STX
  Send_Buf[1] = 0x30;
  Send_Buf[2] = 0x30; //NodeNo
  Send_Buf[3] = 0x30;
  Send_Buf[4] = 0x30; //SubAddr.
  Send_Buf[5] = 0x30;

  Send_Buf[6] ='0'; //CommandText
  Send_Buf[7] ='1';
  Send_Buf[8] ='0';
  Send_Buf[9] ='1';
  Send_Buf[10] ='C';
  Send_Buf[11] ='0';
  
  Send_Buf[12] ='0';
  Send_Buf[13] ='0';
  Send_Buf[14] ='0';
  Send_Buf[15] ='0';

  Send_Buf[16] ='0';
  Send_Buf[17] ='0';

  Send_Buf[18] ='0';
  Send_Buf[19] ='0';
  Send_Buf[20] ='0';
  Send_Buf[21] ='1';

  
  Send_Buf[22] = 0x03; //ETX
  Send_Buf[23] = 0x00;
  int i;
  for(i=1 ; i <= 22 ; i++){
    Send_Buf[23] = Send_Buf[23] ^ Send_Buf[i];
  }
  Send_Buf[24] = 0x00;
  
  struct termios Send_Newtio;
  int SendLen;
  fd = open( Send_Dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  printf("fd:%d\n", fd);
  bzero( &Send_Newtio, sizeof( Send_Newtio ) );		
  Send_Newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;	
  Send_Newtio.c_iflag = IGNPAR;
  Send_Newtio.c_oflag = 0;
  Send_Newtio.c_lflag = ~(ECHO|ICANON);
  Send_Newtio.c_cc[VMIN] = 1;
  Send_Newtio.c_cc[VTIME] = 0;
  
  tcflush(fd,TCIOFLUSH);
  tcsetattr( fd, TCSANOW, &Send_Newtio );
  
  SendLen = 24;
  Send_Res = write( fd, Send_Buf,SendLen);		// Write data to device
  sleep(1);
  Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));
  printf("%d, %s\n", Send_Res, Read_Buf);
  unsigned long temp;

  sscanf(Read_Buf+39, "%lx", &temp) ;
  printf("%lu\n", temp);
  printf("%s\n", Read_Buf+39);
  
  close(fd);								     
}

