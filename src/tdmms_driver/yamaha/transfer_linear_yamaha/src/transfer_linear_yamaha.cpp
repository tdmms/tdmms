// Copyright 2016 by S. Masubuchi

#include <arpa/inet.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <transfer_linear_yamaha/AbsMove.h>
#include <transfer_linear_yamaha/GetCurrentPos.h>
#include <string>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>

#define SERIAL_DATA_LENGTH 4097
#define BAUDRATE B9600

class transfer_linear_yamaha_master {
 public:
  transfer_linear_yamaha_master() {
    int Send_Res;
    snprintf(Send_Dev, sizeof(Send_Dev), "/dev/ttyCom5");

    ros::NodeHandle node;
    cmd_abs_move_ = node.advertiseService(
        "/transfer_linear_yamaha_master/cmd_abs_move",
        &transfer_linear_yamaha_master::cmd_abs_move_Callback, this);
    cmd_stop_ =
        node.subscribe("/transfer_linear_yamaha_master/cmd_stop", 1,
                       &transfer_linear_yamaha_master::cmd_stop_Callback, this);
    cmd_init_ =
        node.subscribe("/transfer_linear_yamaha_master/cmd_init", 1,
                       &transfer_linear_yamaha_master::cmd_init_Callback, this);
    service_getcurpos = node.advertiseService(
        "/stamper_sample_xy_master/get_currpos",
        &transfer_linear_yamaha_master::get_currpos, this);

    Send_Fd = open(Send_Dev, O_RDWR | O_NOCTTY);  // Open device
    if (Send_Fd < 0) {
      ROS_ERROR("Cannot Open Serial Port: %s", Send_Dev);
      exit(-1);
    }

    bzero(&Send_Newtio, sizeof(Send_Newtio));  // Clear new attribute
    Send_Newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD | PARENB |
                          PARODD;  // New attribute assigned
    Send_Newtio.c_iflag = IXON | IXOFF;
    Send_Newtio.c_oflag = 0;
    Send_Newtio.c_lflag = ICANON;
    tcflush(Send_Fd, TCOFLUSH);  // Flush data writen
    if (tcsetattr(Send_Fd, TCSANOW, &Send_Newtio) != 0) {
      ROS_ERROR("Cannot set serial port attributes");
      exit(-1);
    }

    ROS_INFO("Successfully Connected to Yamaha SR1-X");

    snprintf(Send_Buf, sizeof(Send_Buf), "@ALMRST\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);

    snprintf(Send_Buf, sizeof(Send_Buf), "@SRVO 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);
  }

  ~transfer_linear_yamaha_master() { close(Send_Fd); }

  void cmd_stop_Callback(const std_msgs::Empty &emp) {}
  bool cmd_abs_move_Callback(transfer_linear_yamaha::AbsMove::Request &req,
                             transfer_linear_yamaha::AbsMove::Response &res) {
    int Send_Res;
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "@MOVD %4.2f,20\r\n",
             req.targetPose.position.x);
    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }
    while (1) {
      ros::Duration(0.1).sleep();
      memset(Read_Buf, 0x00, sizeof(Read_Buf));
      Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
      if (strstr(Read_Buf, "OK") != NULL) break;
    }
    return true;
  }

  void cmd_init_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "@ORG\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    if (Send_Res == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }
  }

  bool get_currpos(transfer_linear_yamaha::GetCurrentPos::Request &req,
                   transfer_linear_yamaha::GetCurrentPos::Response &res) {
    return true;
  }

 private:
  ros::ServiceServer cmd_abs_move_;
  ros::Subscriber cmd_init_;
  ros::Subscriber cmd_stop_;
  ros::ServiceServer service_getcurpos;
  int Send_Fd;
  char Send_Dev[100];
  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
  struct termios Send_Newtio;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "transfer_linear_yamaha_master");
  transfer_linear_yamaha_master transfer_linear_yamaha_master_;
  ros::spin();
  return 0;
}
