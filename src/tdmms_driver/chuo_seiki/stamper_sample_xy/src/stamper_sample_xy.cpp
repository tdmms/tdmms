// Copyright 2016 by S. Masubuchi

// Two-dimensional materials manufacturing system.
// Device driver for controlling XY sample stage.
// This program is compatible with 2 axis controller driver
// QT-ADM2-35 by Chuo Precision Industrial Co., LTD.

#include <arpa/inet.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <stamper_sample_xy/velocity.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <stamper_sample_xy/GetCurrentPos.h>
#include <string>
#include <iomanip>

#define SERIAL_DATA_LENGTH 4097

class stamper_sample_xy_master {
 public:
  stamper_sample_xy_master() {
    ros::NodeHandle node;
    std::string parameter_name;
    std::string ipaddr;
    int ipport;

    strDelimiter = "\r\n";

    // Load IP Address for communication
    parameter_name = "/stamper_sample_xy/ipaddr";
    if (!node.getParam(parameter_name, ipaddr)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node.getNamespace().c_str());
      exit(-1);
    } else {
      ROS_INFO("Parameter %s : %s", parameter_name.c_str(), ipaddr.c_str());
    }

    parameter_name = "/stamper_sample_xy/ipport";
    if (!node.getParam(parameter_name, ipport)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node.getNamespace().c_str());
      exit(-1);
    } else {
      ROS_INFO("Parameter %s : %s", parameter_name.c_str(), ipaddr.c_str());
    }

    // Opening socket communication port
    sock = socket(AF_INET, SOCK_STREAM, 0);
    server.sin_family = AF_INET;
    server.sin_port = htons(ipport);
    server.sin_addr.s_addr = inet_addr(ipaddr.c_str());
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
      ROS_ERROR("Connection Error");
      exit(-1);
    }

    cmd_stp_move_ = node.subscribe(
        "/stamper_sample_xy_master/cmd_stp_move", 1,
        &stamper_sample_xy_master::cmd_stp_move_Callback, this);
    cmd_jog_move_ = node.subscribe(
        "/stamper_sample_xy_master/cmd_jog_move", 1,
        &stamper_sample_xy_master::cmd_jog_move_Callback, this);
    cmd_abs_move_ = node.subscribe(
        "/stamper_sample_xy_master/cmd_abs_move", 1,
        &stamper_sample_xy_master::cmd_abs_move_Callback, this);
    cmd_stop_     = node.subscribe(
        "/stamper_sample_xy_master/cmd_stop", 1,
        &stamper_sample_xy_master::cmd_stop_Callback, this);
    cmd_init_     = node.subscribe(
        "/stamper_sample_xy_master/cmd_init", 1,
        &stamper_sample_xy_master::cmd_init_Callback, this);
    cmd_status_   = node.subscribe(
        "/stamper_sample_xy_master/cmd_stat", 1,
        &stamper_sample_xy_master::cmd_status_Callback, this);
    cmd_vel_      = node.subscribe(
        "/stamper_sample_xy_master/cmd_vel", 1,
        &stamper_sample_xy_master::cmd_vel_Callback, this);
    stat_pub_     = node.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/status", 1);
    currpos_pub_ = node.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/currpos_stream", 1);
    service       = node.advertiseService(
        "/stamper_sample_xy_master/wait_for_stop",
        &stamper_sample_xy_master::wait_for_stop, this);
    service_getcurpos = node.advertiseService(
        "/stamper_sample_xy_master/get_currpos",
        &stamper_sample_xy_master::get_currpos, this);
  }

  ~stamper_sample_xy_master() {
    close(sock);
  }

  bool sendCommand(const std::string strCommand) {
    strSendBuf = strCommand +strDelimiter;
    if ( write(sock, strSendBuf.c_str(),
               strSendBuf.length()) == -1 ) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      // ROS_INFO("Sent: %s", strCommand.c_str());
    }
  }

  std::string readResponse() {
    read(sock, Read_Buf, SERIAL_DATA_LENGTH);
    return Read_Buf;
  }

  void cmd_stop_Callback(const std_msgs::Empty &emp) {
    sendCommand("L:");
  }

  void cmd_init_Callback(const std_msgs::Empty &emp) {
    sendCommand("H:AB");
  }

  void cmd_jog_move_Callback(const geometry_msgs::Point &msgpnt) {
    if (msgpnt.x > 0.0 && msgpnt.y == 0.0)
      sendCommand("JGO:A+");
    else if (msgpnt.x < 0.0 && msgpnt.y == 0.0)
      sendCommand("JGO:A-");
    else if (msgpnt.x == 0.0 && msgpnt.y > 0.0)
      sendCommand("JGO:B+");
    else if (msgpnt.x == 0.0 && msgpnt.y < 0.0)
      sendCommand("JGO:B-");
  }

  void cmd_abs_move_Callback(const geometry_msgs::Point &msgpnt) {
    std::ostringstream stream;

    stream << "AGO:A" << std::dec << std::setfill('0')
           << std::setprecision(9) << std::right <<  msgpnt.x
           << "B"     << std::dec << std::setfill('0')
           << std::setprecision(9) << std::right <<  msgpnt.y;
    sendCommand(stream.str());
  }

  void cmd_stp_move_Callback(const geometry_msgs::Point &msgpnt) {
    std::ostringstream stream;

    stream << "MGO:A" << std::dec << msgpnt.x
           << "B"     << std::dec << msgpnt.y;
    sendCommand(stream.str());
  }

  void cmd_status_Callback(const geometry_msgs::Point &msgpnt) {
    geometry_msgs::Point currpoint;
    std::string response;
    sendCommand("Q:1");
    ros::Duration(0.02).sleep();

    response = readResponse();
    sscanf(response.data(), "%lf,%lf", & currpoint.x, &currpoint.y);
    stat_pub_.publish(currpoint);
  }

  void cmd_vel_Callback(const stamper_sample_xy::velocity vel) {
    std::ostringstream stream;

    stream << "D:A" << std::noshowpoint << vel.velocity_low << ","
           << std::noshowpoint << vel.velocity_high << ","
           << std::noshowpoint << vel.accl;
    sendCommand(stream.str());

    stream.str("");
    stream.clear(std::stringstream::goodbit);

    stream << "D:B" << std::noshowpoint << vel.velocity_low << ","
           << std::noshowpoint << vel.velocity_high << ","
           << std::noshowpoint << vel.accl;
    sendCommand(stream.str());
  }

  bool wait_for_stop(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &resp) {
    std::string response;
    do {
      sendCommand("Q:2");
      ros::Duration(0.1).sleep();
      response = readResponse();
    }while((response[0] != 'K') || (response[2] != 'K'));
    return true;
  }

  bool get_currpos(stamper_sample_xy::GetCurrentPos::Request &req,
                   stamper_sample_xy::GetCurrentPos::Response &res) {
    int Send_Res;
    std::string response;

    sendCommand("Q:1");
    ros::Duration(0.05).sleep();
    response = readResponse();
    sscanf(response.data(), "%lu,%lu", &res.pos_x, &res.pos_y);
    // ROS_INFO("Recieved Message: %s", response.c_str());
    return true;
  }

  void currpos_stream() {
    geometry_msgs::Point currpoint;
    std::string response;
    sendCommand("Q:1");
    ros::Duration(0.02).sleep();

    response = readResponse();
    sscanf(response.data(), "%lf,%lf", & currpoint.x, &currpoint.y);
    currpos_pub_.publish(currpoint);
  }

 private:
  ros::Subscriber cmd_stp_move_;
  ros::Subscriber cmd_jog_move_;
  ros::Subscriber cmd_abs_move_;
  ros::Subscriber cmd_stop_;
  ros::Subscriber cmd_init_;
  ros::Subscriber cmd_status_;
  ros::Subscriber cmd_vel_;
  ros::Publisher stat_pub_;
  ros::Publisher currpos_pub_;
  ros::ServiceServer service;
  ros::ServiceServer service_getcurpos;
  int sock;
  struct sockaddr_in server;
  std::string strDelimiter;
  std::string strSendBuf;
  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "stamper_sample_xy_master");
  stamper_sample_xy_master stamper_sample_xy_master_;

  ros::Rate loop_rate(1);
  while(ros::ok()) {
    stamper_sample_xy_master_.currpos_stream();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
