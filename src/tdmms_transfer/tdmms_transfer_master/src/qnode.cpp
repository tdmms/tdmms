/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include "../include/tdmms_transfer_master/qnode.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <stamper_yamaha_action/TransferChipAction.h>
#include <adv_dio/dio_write.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_transfer_master {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv) {}

QNode::~QNode() {
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "tdmms_transfer_master");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "tdmms_transfer_master");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

  transfer_valve_open_publisher =
      n.advertise<std_msgs::Empty>("/transfer_valve/cmd_open", 1);
  transfer_valve_close_publisher =
      n.advertise<std_msgs::Empty>("/transfer_valve/cmd_close", 1);

  iai_gripper_left_open =
      n.advertise<std_msgs::Empty>("/iai_gripper/cmd_left_open", 1);
  iai_gripper_left_close =
      n.advertise<std_msgs::Empty>("/iai_gripper/cmd_left_close", 1);
  iai_gripper_right_open =
      n.advertise<std_msgs::Empty>("/iai_gripper/cmd_right_open", 1);
  iai_gripper_right_close =
      n.advertise<std_msgs::Empty>("/iai_gripper/cmd_right_close", 1);

  sample_valve_open =
      n.advertise<std_msgs::Empty>("/sample_stage_valve/cmd_open", 1);
  sample_valve_close =
      n.advertise<std_msgs::Empty>("/sample_stage_valve/cmd_close", 1);
  tape_valve_open =
      n.advertise<std_msgs::Empty>("/tape_stage_valve/cmd_open", 1);
  tape_valve_close =
      n.advertise<std_msgs::Empty>("/tape_stage_valve/cmd_close", 1);

  adv_dio_write = n.serviceClient<adv_dio::dio_write>("/adv_dio/write");
  dio_subscriber =
      n.subscribe("/adv_dio/status", 10, &QNode::dio_callback, this);

  start();
  statvalue.data = 0;
  std_msgs::Empty emp;
  on_DIO_clicked(true);
  return true;
}

void QNode::dio_callback(const std_msgs::UInt16::ConstPtr &Status) {
  std_msgs::Empty emp;

  if ((Status->data & 0x01) && !(statvalue.data & 0x01)) {
    iai_gripper_left_close.publish(emp);
    statvalue.data = statvalue.data ^ 0x01;
  } else if (!(Status->data & 0x01) && (statvalue.data & 0x01)) {
    iai_gripper_left_open.publish(emp);
    statvalue.data = statvalue.data ^ 0x01;
  }

  if (((Status->data >>1) & 0x01) && !((statvalue.data >>1) & 0x01)) {
    iai_gripper_right_close.publish(emp);
    statvalue.data = statvalue.data ^ 0x02;
  } else if (!((Status->data >>1) & 0x01) && ((statvalue.data >>1) & 0x01)) {
    iai_gripper_right_open.publish(emp);
    statvalue.data = statvalue.data ^ 0x02;
  }

  if (((Status->data >>2) & 0x01) && !(statvalue.data >>2) & 0x01) {
    tape_valve_open.publish(emp);
    statvalue.data = statvalue.data ^ 0x04;
  } else if (!((Status->data >>2) & 0x01) && ((statvalue.data >>2) & 0x01)) {
    tape_valve_close.publish(emp);
    statvalue.data = statvalue.data ^ 0x04;
  }
  adv_dio::dio_write write_srv;
  write_srv.request.value = statvalue.data;
  adv_dio_write.call(write_srv);
  ROS_INFO("DI:%d, DO:%d", Status->data, statvalue.data);
}

void QNode::run() {
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to
                         // roslaunch)
}
void QNode::on_Chuck_clicked(bool check) {
  std_msgs::Empty emp;
  if (pushButton_Chuck->isChecked()) {
    ROS_INFO("clicked");
    sample_valve_open.publish(emp);
  } else {
    sample_valve_close.publish(emp);
  }
}

void QNode::on_DIO_clicked(bool check) {
  adv_dio::dio_write write_srv;
  write_srv.request.value = statvalue.data;
  adv_dio_write.call(write_srv);
}

void QNode::on_right_open_clicked(bool check) {
  std_msgs::Empty emp;
  iai_gripper_right_open.publish(emp);
}

void QNode::on_right_grip_clicked(bool check) {}
void QNode::on_right_close_clicked(bool check) {
  std_msgs::Empty emp;
  iai_gripper_right_close.publish(emp);
}
void QNode::on_left_open_clicked(bool check) {
  std_msgs::Empty emp;
  iai_gripper_left_open.publish(emp);
}
void QNode::on_left_grip_clicked(bool check) {}
void QNode::on_left_close_clicked(bool check) {
  std_msgs::Empty emp;
  iai_gripper_left_close.publish(emp);
}

void QNode::log(const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case(Debug) : {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Info) : {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Warn) : {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Error) : {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Fatal) : {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                        new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

}  // namespace tdmms_transfer_master
