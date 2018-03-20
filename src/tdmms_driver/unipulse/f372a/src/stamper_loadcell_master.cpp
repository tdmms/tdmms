// Copyright by S. Masubuchi 2016

#include <ros/ros.h>
#include <fcntl.h>
#include <termios.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#define BAUDRATE B19200
#define SERIAL_DATA_LENGTH 4097

int Send_Fd;
char Send_Dev[SERIAL_DATA_LENGTH];
char Send_Buf[SERIAL_DATA_LENGTH];
char Read_Buf[SERIAL_DATA_LENGTH];
int Send_Res;

void zeroset(std_msgs::Empty emp) {
  tcflush(Send_Fd, TCIOFLUSH);
  snprintf(Send_Buf, sizeof(Send_Buf), "CG\r\n");
  Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
}

int main(int argc, char *argv[]) {
  ros::Publisher stat_pub_;
  ros::Subscriber zero_sub_;
  struct termios Send_Newtio;

  ros::init(argc, argv, "stamper_loadcell");
  ros::NodeHandle node;
  stat_pub_ = node.advertise<std_msgs::Float32>("/stamper_loadcell/value", 1);
  zero_sub_ =
      node.subscribe<std_msgs::Empty>("/stamper_loadcell/zero", 1, &zeroset);

  snprintf(Send_Dev, sizeof(Send_Buf), "/dev/ttyCom3");
  Send_Fd = open(Send_Dev, O_RDWR | O_NOCTTY);  // Open device
  if (Send_Fd < 0) {
    perror(Send_Dev);
    exit(-1);
  }
  bzero(&Send_Newtio, sizeof(Send_Newtio));  // Clear new attribute
  Send_Newtio.c_cflag = PARENB | BAUDRATE | CS8 | CLOCAL |
                        CREAD;  // New attribute assigned
  Send_Newtio.c_iflag = 0;
  Send_Newtio.c_oflag = 0;
  Send_Newtio.c_lflag = ICANON;
  tcflush(Send_Fd, TCIOFLUSH);                         // Flush data writen
  if (tcsetattr(Send_Fd, TCSANOW, &Send_Newtio) != 0)  // Set new attribute
  {
    printf("tcsetattr Error! \n ");
    exit(-1);
  }

  tcflush(Send_Fd, TCIOFLUSH);
  ros::Duration(0.05).sleep();
  tcflush(Send_Fd, TCIOFLUSH);
  ros::Duration(0.05).sleep();

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    std_msgs::Float32 load;
    snprintf(Send_Buf, sizeof(Send_Buf), "RA\r\n");

    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));

    sscanf(Read_Buf, "RA%f", &load.data);

    stat_pub_.publish(load);
    ros::spinOnce();
    loop_rate.sleep();
  }
  close(Send_Fd);
  return 0;
}
