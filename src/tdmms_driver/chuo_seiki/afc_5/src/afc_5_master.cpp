#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

#define BAUDRATE B19200
#define SERIAL_DATA_LENGTH 4097

class afc_5_master {
 public:
  afc_5_master() {
    snprintf(Send_Dev, sizeof(Send_Dev), "/dev/ttyCom0");

    ros::NodeHandle node;
    cmd_chx_ = node.subscribe("/afc_5_master/cmd_chx", 1,
                              &afc_5_master::cmd_chx_Callback, this);
    cmd_sc0_ = node.subscribe("/afc_5_master/cmd_sc0", 1,
                              &afc_5_master::cmd_sc0_Callback, this);

    Send_Fd = open(Send_Dev, O_RDWR | O_NOCTTY);  // Open device
    if (Send_Fd < 0) {
      ROS_ERROR("Cannot Open Serial Port: %s", Send_Dev);
      exit(-1);
    }

    bzero(&Send_Newtio, sizeof(Send_Newtio));  // Clear new attribute
    Send_Newtio.c_cflag =
        BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;  // New attribute assigned
    Send_Newtio.c_iflag = IGNPAR | ICRNL;
    Send_Newtio.c_oflag = 0;
    Send_Newtio.c_lflag = ICANON;

    tcflush(Send_Fd, TCIOFLUSH);
    if (tcsetattr(Send_Fd, TCSANOW, &Send_Newtio) != 0) {
      ROS_ERROR("Cannot set serial port attributes");
      exit(-1);
    }
  }

  ~afc_5_master() { close(Send_Fd); }

  void cmd_chx_Callback(const std_msgs::UInt8 &cmd) {
    tcflush(Send_Fd, TCIOFLUSH);
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "Q\r\n");
    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
    tcflush(Send_Fd, TCIOFLUSH);
    ros::Duration(0.1).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "X%c\r\n", 'A' + cmd.data - 1);
    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
    ros::Duration(0.05).sleep();
    read(Send_Fd, Read_Buf, 256);
    if (Read_Buf[0] != 'K') ROS_WARN("Auto Focus Unit CM Error");
  }

  void cmd_sc0_Callback(const std_msgs::Empty &cmd) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "Q\r\n");
    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }

    ros::Duration(0.1).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "SC0\r\n");
    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
    tcflush(Send_Fd, TCIOFLUSH);
  }

 private:
  ros::Subscriber cmd_chx_;
  ros::Subscriber cmd_sc0_;

  int Send_Fd;
  char Send_Dev[100];
  char Send_Buf[256];
  char Read_Buf[256];
  struct termios Send_Newtio;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "afc_5_master");
  afc_5_master afc_5_master_;
  ros::spin();
  return 0;
}
