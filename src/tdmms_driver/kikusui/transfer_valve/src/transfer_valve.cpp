// Copyright 2016 S. Masubuchi

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
class transfer_valve_master{
 public:
 transfer_valve_master() {
    ros::NodeHandle node;
    std::string ipaddr;
    int ipport;
    std::string parameter_name;

    strDelimiter = "\r\n";
    parameter_name = "/transfer_valve/ipaddr";
    if (!node.getParam(parameter_name, ipaddr)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node.getNamespace().c_str());
      exit(-1);
    } else {
      ROS_INFO("Parameter %s : %s", parameter_name.c_str(), ipaddr.c_str());
    }
    parameter_name = "/transfer_valve/ipport";
    if (!node.getParam(parameter_name, ipport)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node.getNamespace().c_str());
      exit(-1);
    } else {
      ROS_INFO("Parameter %s : %d", parameter_name.c_str(), ipport);
    }

    sock = socket(AF_INET, SOCK_STREAM, 0);
    server.sin_family = AF_INET;
    server.sin_port = htons(ipport);
    server.sin_addr.s_addr = inet_addr(ipaddr.c_str());
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
      ROS_ERROR("Connection Error");
      exit(-1);
    }

    sendCommand("SYST:REM");

    cmd_close_     = node.subscribe(
        "/transfer_valve/cmd_close", 1,
        &transfer_valve_master::cmd_close_Callback, this);

    cmd_open_     = node.subscribe(
        "/transfer_valve/cmd_open", 1,
        &transfer_valve_master::cmd_open_Callback, this);
  }

  ~transfer_valve_master() {
    close(sock);
  }

  bool sendCommand(const std::string strCommand) {
    strSendBuf = strCommand +strDelimiter;
    if ( write(sock, strSendBuf.c_str(),
               strSendBuf.length()) == -1 ) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", strCommand.c_str());
    }
  }

  std::string readResponse() {
    read(sock, Read_Buf, 255);
    return Read_Buf;
  }

  void cmd_open_Callback(const std_msgs::Empty &emp) {
    sendCommand("OUTP 1");
  }

  void cmd_close_Callback(const std_msgs::Empty &emp) {
    sendCommand("OUTP 0");
  }

 private:
  ros::Subscriber cmd_open_;
  ros::Subscriber cmd_close_;
  int sock;
  struct sockaddr_in server;
  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
  std::string strCommand;
  std::string strSendBuf;
  std::string strDelimiter;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "transfer_valve");
  transfer_valve_master transfer_valve_master_;
  ros::spin();
  return 0;
}
