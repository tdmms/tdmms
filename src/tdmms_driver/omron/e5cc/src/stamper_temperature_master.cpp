#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <sys/time.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <csignal>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#define BAUDRATE B9600
#define SERIAL_DATA_LENGTH 4097

int Send_Len;
int Send_Res;
int fd;
char Send_Dev[] = "/dev/ttyUSBOmronTemp";
char Send_Buf[SERIAL_DATA_LENGTH];
char Read_Buf[SERIAL_DATA_LENGTH];

void set_temp(std_msgs::UInt8 temp) {
  Send_Buf[0] = 0x02;  // STX
  Send_Buf[1] = 0x30;
  Send_Buf[2] = 0x30;  // NodeNo
  Send_Buf[3] = 0x30;
  Send_Buf[4] = 0x30;  // SubAddr.
  Send_Buf[5] = 0x30;

  Send_Buf[6] = '0';  // CommandText
  Send_Buf[7] = '1';
  Send_Buf[8] = '0';
  Send_Buf[9] = '2';
  Send_Buf[10] = 'C';
  Send_Buf[11] = '1';

  Send_Buf[12] = '0';
  Send_Buf[13] = '0';
  Send_Buf[14] = '4';
  Send_Buf[15] = 'F';

  Send_Buf[16] = '0';
  Send_Buf[17] = '0';

  Send_Buf[18] = '0';
  Send_Buf[19] = '0';
  Send_Buf[20] = '0';
  Send_Buf[21] = '1';

  sprintf(Send_Buf + 22, "%08X", temp.data);

  Send_Buf[30] = 0x03;
  Send_Buf[31] = 0x00;

  int i;
  for (i = 1; i <= 30; i++) {
    Send_Buf[31] = Send_Buf[31] ^ Send_Buf[i];
  }

  Send_Buf[32] = 0x00;
  tcflush(fd, TCIOFLUSH);
  Send_Res = write(fd, Send_Buf, 32);  // Write data to device
  ros::Duration(0.1).sleep();
  Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));
  unsigned long tempbuf;
}

int main(int argc, char *argv[]) {
  ros::Publisher stat_pub_sv;
  ros::Publisher stat_pub_pv;
  ros::Subscriber temp_sub_;

  struct termios Send_Newtio;
  int SendLen;
  fd = open(Send_Dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  // printf("fd:%d\n", fd);
  bzero(&Send_Newtio, sizeof(Send_Newtio));
  Send_Newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  Send_Newtio.c_iflag = IGNPAR;
  Send_Newtio.c_oflag = 0;
  Send_Newtio.c_lflag = ~(ECHO | ICANON);
  Send_Newtio.c_cc[VMIN] = 1;
  Send_Newtio.c_cc[VTIME] = 0;

  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &Send_Newtio);

  ros::init(argc, argv, "stamper_temperature");
  ros::NodeHandle node;
  stat_pub_pv = node.advertise<std_msgs::UInt8>("/stamper_temperature/pv", 1);
  stat_pub_sv = node.advertise<std_msgs::UInt8>("/stamper_temperature/sv", 1);
  temp_sub_ = node.subscribe<std_msgs::UInt8>("/stamper_temperature/set_temp",
                                              1, &set_temp);

  ros::Rate loop_rate(5);

  Send_Buf[0] = 0x02;  // STX
  Send_Buf[1] = 0x30;
  Send_Buf[2] = 0x30;  // NodeNo
  Send_Buf[3] = 0x30;
  Send_Buf[4] = 0x30;  // SubAddr.
  Send_Buf[5] = 0x30;

  Send_Buf[6] = '3';  // CommandText
  Send_Buf[7] = '0';
  Send_Buf[8] = '0';
  Send_Buf[9] = '5';
  Send_Buf[10] = '0';
  Send_Buf[11] = '0';

  Send_Buf[12] = '0';
  Send_Buf[13] = '1';  //通信書き込み許可
  Send_Buf[14] = 0x03;  // ETX
  Send_Buf[15] = 0x00;
  int i;

  for (i = 1; i <= 14; i++) {
    Send_Buf[15] = Send_Buf[15] ^ Send_Buf[i];
  }

  Send_Res = write(fd, Send_Buf, 24);  // Write data to device
  ros::Duration(0.1).sleep();
  Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));

  Send_Buf[0] = 0x02;  // STX
  Send_Buf[1] = 0x30;
  Send_Buf[2] = 0x30;  // NodeNo
  Send_Buf[3] = 0x30;
  Send_Buf[4] = 0x30;  // SubAddr.
  Send_Buf[5] = 0x30;

  Send_Buf[6] = '3';  // CommandText
  Send_Buf[7] = '0';
  Send_Buf[8] = '0';
  Send_Buf[9] = '5';
  Send_Buf[10] = '0';
  Send_Buf[11] = '4';
  Send_Buf[12] = '0';
  Send_Buf[13] = '1';  //書き込みモード：RAM
  Send_Buf[14] = 0x03;  // ETX
  Send_Buf[15] = 0x00;

  for (i = 1; i <= 14; i++) {
    Send_Buf[15] = Send_Buf[15] ^ Send_Buf[i];
  }

  Send_Res = write(fd, Send_Buf, 24);  // Write data to device
  ros::Duration(0.1).sleep();
  Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));
  // printf("%s\n", Read_Buf);

  Send_Buf[0] = 0x02;  // STX
  Send_Buf[1] = 0x30;
  Send_Buf[2] = 0x30;  // NodeNo
  Send_Buf[3] = 0x30;
  Send_Buf[4] = 0x30;  // SubAddr.
  Send_Buf[5] = 0x30;

  Send_Buf[6] = '3';  // CommandText
  Send_Buf[7] = '0';
  Send_Buf[8] = '0';
  Send_Buf[9] = '5';
  Send_Buf[10] = '0';
  Send_Buf[11] = '1';

  Send_Buf[12] = '0';  //ラン・ストップー＞ストップ
  Send_Buf[13] = '1';
  Send_Buf[14] = 0x03;  // ETX
  Send_Buf[15] = 0x00;

  for (i = 1; i <= 14; i++) {
    Send_Buf[15] = Send_Buf[15] ^ Send_Buf[i];
  }

  Send_Res = write(fd, Send_Buf, 24);  // Write data to device
  ros::Duration(0.1).sleep();
  Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));
  // printf("%s\n", Read_Buf);

  Send_Buf[0] = 0x02;  // STX
  Send_Buf[1] = 0x30;
  Send_Buf[2] = 0x30;  // NodeNo
  Send_Buf[3] = 0x30;
  Send_Buf[4] = 0x30;  // SubAddr.
  Send_Buf[5] = 0x30;

  Send_Buf[6] = '3';  // CommandText
  Send_Buf[7] = '0';
  Send_Buf[8] = '0';
  Send_Buf[9] = '5';
  Send_Buf[10] = '0';
  Send_Buf[11] = '9';

  Send_Buf[12] = '0';
  Send_Buf[13] = '0';  //オートモード
  Send_Buf[14] = 0x03;  // ETX
  Send_Buf[15] = 0x00;

  for (i = 1; i <= 14; i++) {
    Send_Buf[15] = Send_Buf[15] ^ Send_Buf[i];
  }

  Send_Res = write(fd, Send_Buf, 24);  // Write data to device
  ros::Duration(0.1).sleep();
  Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));
  // printf("%s\n", Read_Buf);

  Send_Buf[0] = 0x02;  // STX
  Send_Buf[1] = 0x30;
  Send_Buf[2] = 0x30;  // NodeNo
  Send_Buf[3] = 0x30;
  Send_Buf[4] = 0x30;  // SubAddr.
  Send_Buf[5] = 0x30;

  Send_Buf[6] = '3';  // CommandText
  Send_Buf[7] = '0';
  Send_Buf[8] = '0';
  Send_Buf[9] = '5';
  Send_Buf[10] = '0';
  Send_Buf[11] = '2';
  Send_Buf[12] = '0';
  Send_Buf[13] = '0';
  Send_Buf[14] = 0x03;  // ETX
  Send_Buf[15] = 0x00;

  for (i = 1; i <= 14; i++) {
    Send_Buf[15] = Send_Buf[15] ^ Send_Buf[i];
  }

  Send_Res = write(fd, Send_Buf, 24);  // Write data to device
  ros::Duration(0.1).sleep();
  Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));
  // printf("%s\n", Read_Buf);

  while (ros::ok()) {
    std_msgs::UInt8 temperature;
    Send_Buf[0] = 0x02;  // STX
    Send_Buf[1] = 0x30;
    Send_Buf[2] = 0x30;  // NodeNo
    Send_Buf[3] = 0x30;
    Send_Buf[4] = 0x30;  // SubAddr.
    Send_Buf[5] = 0x30;

    Send_Buf[6] = '0';  // CommandText
    Send_Buf[7] = '1';
    Send_Buf[8] = '0';
    Send_Buf[9] = '1';
    Send_Buf[10] = 'C';
    Send_Buf[11] = '0';

    Send_Buf[12] = '0';
    Send_Buf[13] = '0';
    Send_Buf[14] = '0';
    Send_Buf[15] = '0';

    Send_Buf[16] = '0';
    Send_Buf[17] = '0';

    Send_Buf[18] = '0';
    Send_Buf[19] = '0';
    Send_Buf[20] = '0';
    Send_Buf[21] = '1';

    Send_Buf[22] = 0x03;  // ETX
    Send_Buf[23] = 0x00;
    int i;
    for (i = 1; i <= 22; i++) {
      Send_Buf[23] = Send_Buf[23] ^ Send_Buf[i];
    }

    tcflush(fd, TCIOFLUSH);
    Send_Res = write(fd, Send_Buf, 24);  // Write data to device
    ros::Duration(0.1).sleep();
    Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));
    unsigned long tempbuf;

    sscanf(Read_Buf + 39, "%lx", &tempbuf);
    temperature.data = tempbuf;

    stat_pub_pv.publish(temperature);

    std_msgs::UInt8 temperature_sv;
    Send_Buf[0] = 0x02;  // STX
    Send_Buf[1] = 0x30;
    Send_Buf[2] = 0x30;  // NodeNo
    Send_Buf[3] = 0x30;
    Send_Buf[4] = 0x30;  // SubAddr.
    Send_Buf[5] = 0x30;

    Send_Buf[6] = '0';  // CommandText
    Send_Buf[7] = '1';
    Send_Buf[8] = '0';
    Send_Buf[9] = '1';
    Send_Buf[10] = 'C';
    Send_Buf[11] = '0';

    Send_Buf[12] = '0';
    Send_Buf[13] = '0';
    Send_Buf[14] = '0';
    Send_Buf[15] = '2';

    Send_Buf[16] = '0';
    Send_Buf[17] = '0';

    Send_Buf[18] = '0';
    Send_Buf[19] = '0';
    Send_Buf[20] = '0';
    Send_Buf[21] = '1';

    Send_Buf[22] = 0x03;  // ETX
    Send_Buf[23] = 0x00;

    for (i = 1; i <= 22; i++) {
      Send_Buf[23] = Send_Buf[23] ^ Send_Buf[i];
    }

    tcflush(fd, TCIOFLUSH);
    Send_Res = write(fd, Send_Buf, 24);  // Write data to device
    ros::Duration(0.1).sleep();
    Send_Res = read(fd, Read_Buf, sizeof(Read_Buf));

    sscanf(Read_Buf + 39, "%lx", &tempbuf);
    temperature_sv.data = tempbuf;

    stat_pub_sv.publish(temperature_sv);

    ros::spinOnce();
    loop_rate.sleep();
  }
  close(fd);
  return 0;
}
