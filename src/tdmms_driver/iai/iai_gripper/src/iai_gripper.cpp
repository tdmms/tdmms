// Copyright 2016 by S. Masubuchi

#include <arpa/inet.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <string>
#include <iomanip>

#define SERIAL_DATA_LENGTH 4097

class iai_gripper{ 
 public:
  iai_gripper() {
    ros::NodeHandle node;
    std::string parameter_name;
    std::string ipaddr;
    int ipport;

    strDelimiter = "\r\n";

    sock = socket(AF_INET, SOCK_STREAM, 0);
    server.sin_family = AF_INET;
    server.sin_port = htons(64516);
    server.sin_addr.s_addr = inet_addr("192.168.0.189");
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
      ROS_ERROR("Connection Error");
      exit(-1);
    }
    ROS_INFO("Successfully Connected to IAI_MSEL Controller");

    ros::Duration(0.2).sleep();

    // Echo check
    snprintf(Send_Buf, sizeof(Send_Buf), "!992001234567890@@\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("Echo: %s", Read_Buf);

    // Servo ON
    ros::Duration(0.1).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "!9925310@@\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("Echo: %s", Read_Buf);

    // Initialize gripper
    ros::Duration(0.3).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "!992530A@@\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("Echo: %s", Read_Buf);

    ros::Duration(3).sleep();
    
    cmd_left_close_ =
        node.subscribe("/iai_gripper/cmd_left_close", 1,
                       &iai_gripper::cmd_left_close_Callback, this);

    cmd_left_open_ =
        node.subscribe("/iai_gripper/cmd_left_open", 1,
                       &iai_gripper::cmd_left_open_Callback, this);

    cmd_right_close_ =
        node.subscribe("/iai_gripper/cmd_right_close", 1,
                       &iai_gripper::cmd_right_close_Callback, this);

    cmd_right_open_ =
        node.subscribe("/iai_gripper/cmd_right_open", 1,
                       &iai_gripper::cmd_right_open_Callback, this);
  }

  ~iai_gripper() {
    // Servo OFF
    ros::Duration(0.1).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "!9925311@@\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("Echo: %s", Read_Buf);

    close(sock);
  }

  bool sendCommand(const std::string strCommand) {
    strSendBuf = strCommand + strDelimiter;
    if (write(sock, strSendBuf.c_str(), strSendBuf.length()) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", strCommand.c_str());
    }
  }

  std::string readResponse() {
    read(sock, Read_Buf, SERIAL_DATA_LENGTH);
    return Read_Buf;
  }

  void cmd_left_open_Callback(const std_msgs::Empty &emp) {
    sendCommand("!992530B@@");
    readResponse();
  }

  void cmd_left_close_Callback(const std_msgs::Empty &emp) {
    sendCommand("!992530C@@");
    readResponse();
  }

  void cmd_right_close_Callback(const std_msgs::Empty &emp) {
    sendCommand("!992530D@@");
    readResponse();
  }
  void cmd_right_open_Callback(const std_msgs::Empty &emp) {
    sendCommand("!992530E@@");
    readResponse();
  }


 private:
  int sock;
  struct sockaddr_in server;
  std::string strDelimiter;
  std::string strSendBuf;
  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
  int Send_Res;

  ros::Subscriber cmd_left_open_;
  ros::Subscriber cmd_left_close_;
  ros::Subscriber cmd_right_open_;
  ros::Subscriber cmd_right_close_;

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "exfoliator_iai_master");
  iai_gripper exfoliator_iai_master_;
  ros::spin();
  return 0;
}
