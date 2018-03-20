// Copyright 2016 by S. Masubuchi

#include <arpa/inet.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <stamper_yamaha/AbsMove.h>
#include <stamper_yamaha/GetCurrentPos.h>
#include <stamper_yamaha/PointNo.h>
#include <stamper_yamaha/PalletPos.h>
#include <string>
#include <iomanip>

#define SERIAL_DATA_LENGTH 4097

class stamper_yamaha_master {
 public:
  stamper_yamaha_master() {
    ros::NodeHandle node;
    std::string parameter_name;
    std::string ipaddr;
    int ipport;

    strDelimiter = "\r\n";
    parameter_name = "/stamper_yamaha/ipaddr";
    if (!node.getParam(parameter_name, ipaddr)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node.getNamespace().c_str());
      exit(-1);
    } else {
      ROS_INFO("Parameter %s : %s", parameter_name.c_str(), ipaddr.c_str());
    }
    parameter_name = "/stamper_yamaha/ipport";
    if (!node.getParam(parameter_name, ipport)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node.getNamespace().c_str());
      exit(-1);
    } else {
      ROS_INFO("Parameter %s : %s", parameter_name.c_str(), ipaddr.c_str());
    }

    sock = socket(AF_INET, SOCK_STREAM, 0);
    server.sin_family = AF_INET;
    server.sin_port = htons(ipport);
    server.sin_addr.s_addr = inet_addr(ipaddr.c_str());
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
      ROS_ERROR("Connection Error");
      exit(-1);
    }

    ros::Duration(0.2).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    if (strstr(Read_Buf, "Welcome to RCX340") == NULL) {
      ROS_ERROR("Connection Error");
      exit(-1);
    }

    ROS_INFO("Successfully Connected to Yamaha RCX340");

    snprintf(Send_Buf, sizeof(Send_Buf), "@ALMRST\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);

    snprintf(Send_Buf, sizeof(Send_Buf), "@TORQUE (1)=20\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);

    snprintf(Send_Buf, sizeof(Send_Buf), "@TORQUE (2)=20\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);

    snprintf(Send_Buf, sizeof(Send_Buf), "@TORQUE (3)=10\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);

    snprintf(Send_Buf, sizeof(Send_Buf), "@TORQUE (4)=10\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);

    snprintf(Send_Buf, sizeof(Send_Buf), "@SERVO ON\r\n");
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.1).sleep();
    memset(Read_Buf, 0x00, sizeof(Read_Buf));
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);

    cmd_stop_ = node.subscribe("/stamper_yamaha_master/cmd_stop", 1,
                               &stamper_yamaha_master::cmd_stop_Callback, this);
    service_ptp = node.advertiseService(
        "/stamper_yamaha_master/cmd_ptp_move",
        &stamper_yamaha_master::cmd_ptp_move_Callback, this);
    service = node.advertiseService(
        "/stamper_yamaha_master/cmd_abs_move_linear",
        &stamper_yamaha_master::cmd_abs_move_linear_Callback, this);
    service_linear = node.advertiseService(
        "/stamper_yamaha_master/cmd_linear_move",
        &stamper_yamaha_master::cmd_linear_move_Callback, this);
    service_pallet = node.advertiseService(
        "/stamper_yamaha_master/cmd_pallet_move",
        &stamper_yamaha_master::cmd_pallet_move_Callback, this);
    service_getcurpos =
        node.advertiseService("/stamper_sample_xy_master/get_currpos",
                              &stamper_yamaha_master::get_currpos, this);
  }

  ~stamper_yamaha_master() { close(sock); }

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

  void cmd_stop_Callback(const std_msgs::Empty &emp) {
    unsigned char stopcmd = 0x03;
    Send_Res = write(sock, &stopcmd, sizeof(stopcmd));
    ros::Duration(0.1).sleep();
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
  }

  bool cmd_linear_move_Callback(stamper_yamaha::PointNo::Request &req,
                                stamper_yamaha::PointNo::Response &res) {
    snprintf(Send_Buf, sizeof(Send_Buf), "@MOVE L,P%d,S=%d\r\n",
             req.pointno.data, req.velocity.data);
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ROS_INFO("%s", Send_Buf);
    while (1) {
      ros::Duration(0.1).sleep();
      memset(Read_Buf, 0x00, sizeof(Read_Buf));
      Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
      if (strstr(Read_Buf, "END") != NULL) break;
    }
  }

  bool cmd_pallet_move_Callback(stamper_yamaha::PalletPos::Request &req,
                                stamper_yamaha::PalletPos::Response &res) {
    snprintf(Send_Buf, sizeof(Send_Buf), "@PMOVE (%d,%d),S=20\r\n",
             req.pallet.data, req.no.data);
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ROS_INFO("%s", Send_Buf);
    while (1) {
      ros::Duration(0.1).sleep();
      memset(Read_Buf, 0x00, sizeof(Read_Buf));
      Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
      if (strstr(Read_Buf, "END") != NULL) break;
    }
  }

  bool cmd_ptp_move_Callback(stamper_yamaha::PointNo::Request &req,
                             stamper_yamaha::PointNo::Response &res) {
    snprintf(Send_Buf, sizeof(Send_Buf), "@MOVE P,P%d,S=%d\r\n",
             req.pointno.data, req.velocity.data);
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ROS_INFO("%s", Send_Buf);
    while (1) {
      ros::Duration(0.1).sleep();
      memset(Read_Buf, 0x00, sizeof(Read_Buf));
      Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
      if (strstr(Read_Buf, "END") != NULL) break;
    }
  }

  bool cmd_abs_move_linear_Callback(stamper_yamaha::AbsMove::Request &req,
                                    stamper_yamaha::AbsMove::Response &res) {
    snprintf(Send_Buf, sizeof(Send_Buf),
             "@P501=%3.3f %3.3f %3.3f %3.3f 0.0 0.0 1 0 0\r\n",
             req.targetPose.position.x, req.targetPose.position.y,
             req.targetPose.position.z,
             req.targetPose.orientation.w);  // Degrees
    ROS_INFO("%s", Send_Buf);
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.02).sleep();
    Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
    ROS_INFO("%s", Read_Buf);
    snprintf(Send_Buf, sizeof(Send_Buf), "@MOVE P,P501,S=%d\r\n",
             req.velocity.data);
    ROS_INFO("%s", Send_Buf);
    Send_Res = write(sock, Send_Buf, strlen(Send_Buf));
    while (1) {
      ros::Duration(0.1).sleep();
      memset(Read_Buf, 0x00, sizeof(Read_Buf));
      Send_Res = read(sock, Read_Buf, sizeof(Read_Buf));
      if (strstr(Read_Buf, "END") != NULL) break;
    }
  }

  bool get_currpos(stamper_yamaha::GetCurrentPos::Request &req,
                   stamper_yamaha::GetCurrentPos::Response &res) {}

 private:
  ros::Subscriber cmd_stop_;
  ros::ServiceServer service;
  ros::ServiceServer service_getcurpos;
  ros::ServiceServer service_ptp;
  ros::ServiceServer service_linear;
  ros::ServiceServer service_pallet;
  int sock;
  struct sockaddr_in server;
  std::string strDelimiter;
  std::string strSendBuf;
  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
  int Send_Res;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "stamper_yamaha_master");
  stamper_yamaha_master stamper_yamaha_master_;
  ros::spin();
  return 0;
}
