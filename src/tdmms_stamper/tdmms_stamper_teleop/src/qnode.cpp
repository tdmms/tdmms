// Copyright 2016 by S. Masubuchi

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>

#include <stamper_sample_z/velocity.h>
#include <stamper_sample_xy/velocity.h>
#include <stamper_sample_theta/velocity.h>
#include <stamper_om_xy/velocity.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <stamper_sample_z/GetCurrentPos.h>
#include <stamper_sample_xy/GetCurrentPos.h>
#include <stamper_om_xy/GetCurrentPos.h>
#include <keyence_master/Status.h>

#include <QMessageBox>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>

#include "../include/tdmms_stamper_teleop/qnode.hpp"
namespace tdmms_stamper_teleop {

QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv) {}

QNode::~QNode() {
  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "tdmms_stamper_teleop");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();
  ros::NodeHandle n;
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  stamper_currtemp_subscriber = n.subscribe("/stamper_temperature/value", 1,
                                            &QNode::currtemp_Callback, this);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "tdmms_stamper_teleop");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();
  ros::NodeHandle n;
  stamper_currtemp_subscriber = n.subscribe("/stamper_temperature/pv", 1,
                                            &QNode::currtemp_Callback, this);
  stamper_currtemp_sv_subscriber = n.subscribe(
      "/stamper_temperature/sv", 1, &QNode::currtemp_sv_Callback, this);
  stamper_currload_subscriber = n.subscribe("/stamper_loadcell/value", 1,
                                            &QNode::currload_Callback, this);
  stamper_keyence_subscriber =
      n.subscribe("/keyence_master/status", 1, &QNode::keyence_Callback, this);

  stamper_sample_xy_subscriber =
      n.subscribe("/stamper_sample_xy_master/currpos_stream", 1,
                  &QNode::currpos_sample_xy_Callback, this);
  stamper_om_xy_subscriber =
      n.subscribe("/stamper_om_xy_master/currpos_stream", 1,
                  &QNode::currpos_om_xy_Callback, this);
  
  stamper_loadcell_zero_publisher =
      n.advertise<std_msgs::Empty>("/stamper_loadcell/zero", 1);
  stamper_temperature_set_publisher =
      n.advertise<std_msgs::UInt8>("/stamper_temperature/set_temp", 1);
  keyence_om_fwd_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_om_fwd", 1);
  keyence_om_back_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_om_back", 1);
  keyence_mask_fwd_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_mask_fwd", 1);
  keyence_mask_back_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_mask_back", 1);
  keyence_sample_cool_on_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_cool_on", 1);
  keyence_sample_cool_off_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_cool_off", 1);
  keyence_sample_blow_on_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_blow_on", 1);
  keyence_sample_blow_off_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_blow_off", 1);
  keyence_stage_lock_on_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_stage_lock_on", 1);
  keyence_stage_lock_off_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_stage_lock_off", 1);
  keyence_sample_vacuum_on_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_vacuum_on", 1);
  keyence_sample_vacuum_off_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_vacuum_off", 1);
  keyence_mask_chuck_on_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_mask_chuck_on", 1);
  keyence_mask_chuck_off_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_mask_chuck_off", 1);
  keyence_status_client =
      n.serviceClient<keyence_master::Status>("/keyence_master/get_status");

  stamper_nic_100a_cmd_on =
      n.advertise<std_msgs::Byte>("/stamper_nic_100a/cmd_on", 1);
  stamper_nic_100a_cmd_off =
      n.advertise<std_msgs::Byte>("/stamper_nic_100a/cmd_off", 1);
  stamper_nic_100a_cmd_inten =
      n.advertise<std_msgs::UInt8>("/stamper_nic_100a/cmd_inten", 1);

  stamper_sample_z_cmd_vel =
      n.advertise<stamper_sample_z::velocity>("/stamper_sample_z_master/cmd_vel", 1);
  stamper_sample_z_cmd_jog = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_z_master/cmd_jog_move", 1);
  stamper_sample_z_cmd_abs = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_z_master/cmd_abs_move", 1);
  stamper_sample_z_cmd_stop =
      n.advertise<std_msgs::Empty>("/stamper_sample_z_master/cmd_stop", 1);
  stamper_sample_z_cmd_home =
      n.advertise<std_msgs::Empty>("/stamper_sample_z_master/cmd_init", 1);
  stamper_sample_z_cmd_stp = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_z_master/cmd_stp_move", 1);
  stamper_sample_z_wait_for_stop_client = n.serviceClient<std_srvs::Empty>(
      "/stamper_sample_z_master/wait_for_stop");
  stamper_sample_z_get_current_pos_client =
      n.serviceClient<stamper_sample_z::GetCurrentPos>(
          "/stamper_sample_z_master/get_currpos");

  stamper_sample_xy_cmd_vel =
      n.advertise<stamper_sample_xy::velocity>("/stamper_sample_xy_master/cmd_vel", 1);
  stamper_sample_xy_cmd_jog = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_xy_master/cmd_jog_move", 1);
  stamper_sample_xy_cmd_abs = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_xy_master/cmd_abs_move", 1);
  stamper_sample_xy_cmd_stop =
      n.advertise<std_msgs::Empty>("/stamper_sample_xy_master/cmd_stop", 1);
  stamper_sample_xy_cmd_stp = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_xy_master/cmd_stp_move", 1);
  stamper_sample_xy_cmd_home =
      n.advertise<std_msgs::Empty>("/stamper_sample_xy_master/cmd_init", 1);
  stamper_sample_xy_get_current_pos_client =
      n.serviceClient<stamper_sample_xy::GetCurrentPos>(
          "/stamper_sample_xy_master/get_currpos");
  stamper_sample_theta_cmd_vel =
      n.advertise<stamper_sample_theta::velocity>("/stamper_sample_theta_master/cmd_vel", 1);
  stamper_sample_theta_cmd_jog = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_theta_master/cmd_jog_move", 1);
  stamper_sample_theta_cmd_abs = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_theta_master/cmd_abs_move", 1);
  stamper_sample_theta_cmd_stop =
      n.advertise<std_msgs::Empty>("/stamper_sample_theta_master/cmd_stop", 1);

  stamper_om_xy_cmd_vel =
      n.advertise<stamper_om_xy::velocity>("/stamper_om_xy_master/cmd_vel", 1);
  stamper_om_xy_cmd_jog = n.advertise<geometry_msgs::Point>(
      "/stamper_om_xy_master/cmd_jog_move", 1);
  stamper_om_xy_cmd_abs = n.advertise<geometry_msgs::Point>(
      "/stamper_om_xy_master/cmd_abs_move", 1);
  stamper_om_xy_cmd_stop =
      n.advertise<std_msgs::Empty>("/stamper_om_xy_master/cmd_stop", 1);
  stamper_om_xy_cmd_stp = n.advertise<geometry_msgs::Point>(
      "/stamper_om_xy_master/cmd_stp_move", 1);
  stamper_om_xy_cmd_home =
      n.advertise<std_msgs::Empty>("/stamper_om_xy_master/cmd_init", 1);
  stamper_om_xy_get_currrent_pos_client =
      n.serviceClient<stamper_om_xy::GetCurrentPos>(
          "/stamper_om_xy_master/get_currpos");

  stamper_nikon_cmd_revolv_next =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_revolv_next", 1);
  stamper_nikon_cmd_revolv_prev =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_revolv_prev", 1);
  stamper_nikon_cmd_revolv_set =
      n.advertise<std_msgs::UInt8>("/stamper_nikon_master/cmd_revolv_set", 1);
  stamper_nikon_cmd_z_u1 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_u1", 1);
  stamper_nikon_cmd_z_u10 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_u10", 1);
  stamper_nikon_cmd_z_u20 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_u20", 1);
  stamper_nikon_cmd_z_u200 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_u200", 1);
  stamper_nikon_cmd_z_u2000 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_u2000", 1);
  stamper_nikon_cmd_z_d1 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_d1", 1);
  stamper_nikon_cmd_z_d10 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_d10", 1);
  stamper_nikon_cmd_z_d20 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_d20", 1);
  stamper_nikon_cmd_z_d200 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_d200", 1);
  stamper_nikon_cmd_z_d2000 =
      n.advertise<std_msgs::Empty>("/stamper_nikon_master/cmd_z_d2000", 1);
  stamper_nikon_cmd_z_abs =
      n.advertise<std_msgs::UInt32>("/stamper_nikon_master/cmd_z_abs", 1);
  start();

  ros::Duration(1).sleep();
  keyence_master::Status stat;
  keyence_status_client.call(stat);
  pushButton_Mask_Chuck_On->setChecked(stat.response.mask_chuck);
  pushButton_Sample_Cool_On->setChecked(stat.response.sample_cool);
  pushButton_Sample_Blow_On->setChecked(stat.response.sample_blow);
  pushButton_Sample_Vacuum_On->setChecked(stat.response.sample_chuck);
  pushButton_Stage_Lock_On->setChecked(stat.response.kyumen_chuck);

  return true;
}

void QNode::run() {
  ros::Rate loop_rate(1);
  int count = 0;

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();
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
  Q_EMIT loggingUpdated();
}

void QNode::keyence_Callback(const keyence_master::Status_msg &stat) {
}

void QNode::currtemp_Callback(const std_msgs::UInt8 &temp) {
  lcdNumber_Temperature->display(temp.data);
}

void QNode::currtemp_sv_Callback(const std_msgs::UInt8 &temp) {
  lcdNumber_Temperature_sv->display(temp.data);
}

void QNode::currload_Callback(const std_msgs::Float32 &load) {
  lcdNumber_Load->display(load.data);
}

void QNode::currpos_sample_xy_Callback(const geometry_msgs::Point &currpoint){
  lcdNumber_Sample_X->display(currpoint.x);
  lcdNumber_Sample_Y->display(currpoint.y);
}

void QNode::currpos_om_xy_Callback(const geometry_msgs::Point &currpoint){
  lcdNumber_OM_X->display(currpoint.x);
  lcdNumber_OM_Y->display(currpoint.y);
}

void QNode::on_OM_Forward_Button_clicked(bool check) {
  std_msgs::Empty emp;
  keyence_om_fwd_publisher.publish(emp);
}

void QNode::on_OM_Back_Button_clicked(bool check) {
  std_msgs::Empty emp;
  keyence_om_back_publisher.publish(emp);
}

void QNode::on_Mask_Forward_Button_clicked(bool check) {
  //安全のため一旦無効に
  std_msgs::Empty emp;
  // keyence_mask_fwd_publisher.publish(emp);
}

void QNode::on_Mask_Back_Button_clicked(bool check) {
  std_msgs::Empty emp;
  //安全のため一旦無効に
  // keyence_mask_back_publisher.publish(emp);
}

void QNode::on_Sample_Cool_On_Button_clicked(bool check) {
  std_msgs::Empty emp;
  if (pushButton_Sample_Cool_On->isChecked()) {
    keyence_sample_cool_on_publisher.publish(emp);
    pushButton_Sample_Cool_On->setChecked(true);
  } else {
    keyence_sample_cool_off_publisher.publish(emp);
    pushButton_Sample_Cool_On->setChecked(false);
  }
}

void QNode::on_Sample_Cool_Off_Button_clicked(bool check) {
}

void QNode::on_Sample_Blow_Off_Button_clicked(bool check) {
}

void QNode::on_Sample_Blow_On_Button_clicked(bool check) {
  std_msgs::Empty emp;
  if (pushButton_Sample_Blow_On->isChecked()) {
    keyence_sample_blow_on_publisher.publish(emp);
    pushButton_Sample_Blow_On->setChecked(true);
  } else {
    keyence_sample_blow_off_publisher.publish(emp);
    pushButton_Sample_Blow_On->setChecked(false);
  }
}

void QNode::on_Stage_Lock_Off_Button_clicked(bool check) {
}

void QNode::on_Stage_Lock_On_Button_clicked(bool check) {
  std_msgs::Empty emp;
  if (pushButton_Stage_Lock_On->isChecked()) {
    keyence_stage_lock_on_publisher.publish(emp);
    pushButton_Stage_Lock_On->setChecked(true);
  } else {
    keyence_stage_lock_off_publisher.publish(emp);
    pushButton_Stage_Lock_On->setChecked(false);
  }
}

void QNode::on_Sample_Vacuum_Off_Button_clicked(bool check) {
}

void QNode::on_Sample_Vacuum_On_Button_clicked(bool check) {
  std_msgs::Empty emp;
  if (pushButton_Sample_Vacuum_On->isChecked()) {
    keyence_sample_vacuum_on_publisher.publish(emp);
    pushButton_Sample_Vacuum_On->setChecked(true);
  } else {
    keyence_sample_vacuum_off_publisher.publish(emp);
    pushButton_Sample_Vacuum_On->setChecked(false);
  }
}

void QNode::on_Mask_Chuck_On_Button_clicked(bool check) {
  std_msgs::Empty emp;
  if (pushButton_Mask_Chuck_On->isChecked()) {
    keyence_mask_chuck_on_publisher.publish(emp);
    pushButton_Mask_Chuck_On->setChecked(true);
  } else {
    keyence_mask_chuck_off_publisher.publish(emp);
    pushButton_Mask_Chuck_On->setChecked(false);
  }
}

void QNode::on_Mask_Chuck_Off_Button_clicked(bool check) {
}

void QNode::on_Loadcell_ZeroSet_Button_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_loadcell_zero_publisher.publish(emp);
}

void QNode::on_Temperature_valueChanged() {
  std_msgs::UInt8 temp;
  temp.data = spinBox_Temperature->value();
  stamper_temperature_set_publisher.publish(temp);
}

void QNode::on_Sample_XY_X_Minus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 1;
  pnt.y = 0;
  stamper_sample_xy_cmd_jog.publish(pnt);
}

void QNode::on_Sample_XY_X_Minus_released() {
  std_msgs::Empty emp;
  stamper_sample_xy_cmd_stop.publish(emp);
}

void QNode::on_Sample_XY_X_Plus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = -1;
  pnt.y = 0;
  stamper_sample_xy_cmd_jog.publish(pnt);
}

void QNode::on_Sample_XY_X_Plus_released() {
  std_msgs::Empty emp;
  stamper_sample_xy_cmd_stop.publish(emp);
}

void QNode::on_Sample_XY_Y_Minus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = -1;
  stamper_sample_xy_cmd_jog.publish(pnt);
}

void QNode::on_Sample_XY_Y_Minus_released() {
  std_msgs::Empty emp;
  stamper_sample_xy_cmd_stop.publish(emp);
}

void QNode::on_Sample_XY_Y_Plus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = 1;
  stamper_sample_xy_cmd_jog.publish(pnt);
}

void QNode::on_Sample_XY_Y_Plus_released() {
  std_msgs::Empty emp;
  stamper_sample_xy_cmd_stop.publish(emp);
}

void QNode::on_Sample_XY_Home_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_sample_xy_cmd_home.publish(emp);
}

void QNode::on_OM_XY_Home_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_om_xy_cmd_home.publish(emp);
}

void QNode::on_Sample_Z_Minus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = -1;
  pnt.y = 0;
  stamper_sample_z_cmd_jog.publish(pnt);
}

void QNode::on_Sample_Z_Minus_released() {
  std_msgs::Empty emp;
  stamper_sample_z_cmd_stop.publish(emp);
}

void QNode::on_Sample_Z_Plus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 1;
  pnt.y = 0;
  stamper_sample_z_cmd_jog.publish(pnt);
}

void QNode::on_Sample_Z_Plus_released() {
  std_msgs::Empty emp;
  stamper_sample_z_cmd_stop.publish(emp);
}

void QNode::on_Sample_Z_Home_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_sample_z_cmd_home.publish(emp);
}

void QNode::on_Sample_Theta_Minus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = -1;
  pnt.y = 0;
  stamper_sample_theta_cmd_jog.publish(pnt);
}

void QNode::on_Sample_Theta_Minus_released() {
  std_msgs::Empty emp;
  stamper_sample_theta_cmd_stop.publish(emp);
}

void QNode::on_Sample_Theta_Plus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 1;
  pnt.y = 0;
  stamper_sample_theta_cmd_jog.publish(pnt);
}

void QNode::on_Sample_Theta_Plus_released() {
  std_msgs::Empty emp;
  stamper_sample_theta_cmd_stop.publish(emp);
}

void QNode::on_OM_XY_X_Minus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 1;
  pnt.y = 0;
  stamper_om_xy_cmd_jog.publish(pnt);
}

void QNode::on_OM_XY_X_Minus_released() {
  std_msgs::Empty emp;
  stamper_om_xy_cmd_stop.publish(emp);
}

void QNode::on_OM_XY_X_Plus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = -1;
  pnt.y = 0;
  stamper_om_xy_cmd_jog.publish(pnt);
}
void QNode::on_OM_XY_X_Plus_released() {
  std_msgs::Empty emp;
  stamper_om_xy_cmd_stop.publish(emp);
}

void QNode::on_OM_XY_Y_Minus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = -1;
  stamper_om_xy_cmd_jog.publish(pnt);
}

void QNode::on_OM_XY_Y_Minus_released() {
  std_msgs::Empty emp;
  stamper_om_xy_cmd_stop.publish(emp);
}

void QNode::on_OM_XY_Y_Plus_pressed() {
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = 1;
  stamper_om_xy_cmd_jog.publish(pnt);
}

void QNode::on_OM_XY_Y_Plus_released() {
  std_msgs::Empty emp;
  stamper_om_xy_cmd_stop.publish(emp);
}

void QNode::on_Sample_XY_X_Plus_Stp_clicked(bool check) {
  stamper_sample_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_xy_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = -spinBox_Sample_XY_STP->value();
  pnt.y = 0;
  stamper_sample_xy_cmd_stp.publish(pnt);
}

void QNode::on_Sample_XY_X_Minus_Stp_clicked(bool check) {
  stamper_sample_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_xy_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = spinBox_Sample_XY_STP->value();
  pnt.y = 0;
  stamper_sample_xy_cmd_stp.publish(pnt);
}

void QNode::on_Sample_XY_Y_Plus_Stp_clicked(bool check) {
  stamper_sample_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_xy_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = spinBox_Sample_XY_STP->value();
  stamper_sample_xy_cmd_stp.publish(pnt);
}

void QNode::on_Sample_XY_Y_Minus_Stp_clicked(bool check) {
  stamper_sample_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_xy_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = -spinBox_Sample_XY_STP->value();
  stamper_sample_xy_cmd_stp.publish(pnt);
}

void QNode::on_Sample_Z_Point0_clicked(bool check) {
  stamper_sample_z::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel);

  geometry_msgs::Point pnt;
  pnt.x = 5000;
  pnt.y = 0;
  stamper_sample_z_cmd_abs.publish(pnt);
}

void QNode::on_Sample_Z_Point1_clicked(bool check) {
  stamper_sample_z::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = 605000;
  pnt.y = 0;
  stamper_sample_z_cmd_abs.publish(pnt);
}

void QNode::on_Sample_Z_Point2_clicked(bool check) {
  stamper_sample_z::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = 585000;
  pnt.y = 0;
  stamper_sample_z_cmd_abs.publish(pnt);
}

void QNode::on_Sample_Z_Plus_Stp_clicked(bool check) {
  stamper_sample_z::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = spinBox_Sample_Z_STP->value();
  pnt.y = 0;
  stamper_sample_z_cmd_stp.publish(pnt);
}

void QNode::on_Sample_Z_Minus_Stp_clicked(bool check) {
  stamper_sample_z::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = -spinBox_Sample_Z_STP->value();
  pnt.y = 0;
  stamper_sample_z_cmd_stp.publish(pnt);
}

void QNode::on_OM_XY_Y_Plus_Stp_clicked(bool check) {
  stamper_om_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = spinBox_OM_XY_STP->value();
  stamper_om_xy_cmd_stp.publish(pnt);
}

void QNode::on_OM_XY_Y_Minus_Stp_clicked(bool check) {
  stamper_om_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel);
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = -spinBox_OM_XY_STP->value();
  stamper_om_xy_cmd_stp.publish(pnt);
}

void QNode::on_OM_XY_X_Plus_Stp_clicked(bool check) {
  stamper_om_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel);

  geometry_msgs::Point pnt;
  pnt.x = -spinBox_OM_XY_STP->value();
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);
}

void QNode::on_OM_XY_X_Minus_Stp_clicked(bool check) {
  stamper_om_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel);

  geometry_msgs::Point pnt;
  pnt.x = spinBox_OM_XY_STP->value();
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);
}

void QNode::on_OM_XY_Center_clicked(bool check) {
  geometry_msgs::Point pnt;
  pnt.x = 750000;
  pnt.y = 750000;
  stamper_om_xy_cmd_abs.publish(pnt);
}

void QNode::on_Sample_XY_Center_clicked(bool check) {
  geometry_msgs::Point pnt;
  pnt.x = 750000;
  pnt.y = 750000;
  stamper_sample_xy_cmd_abs.publish(pnt);
}

void QNode::on_OM_Revolv_Next_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_revolv_next.publish(emp);
}

void QNode::on_OM_Revolv_Prev_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_revolv_prev.publish(emp);
}

void QNode::on_OM_z_U1_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_u1.publish(emp);
}

void QNode::on_OM_z_U10_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_u10.publish(emp);
}

void QNode::on_OM_z_U20_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_u20.publish(emp);
}

void QNode::on_OM_z_U200_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_u200.publish(emp);
}

void QNode::on_OM_z_U2000_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_u2000.publish(emp);
}

void QNode::on_OM_z_D1_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_d1.publish(emp);
}

void QNode::on_OM_z_D10_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_d10.publish(emp);
}

void QNode::on_OM_z_D20_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_d20.publish(emp);
}

void QNode::on_OM_z_D200_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_d200.publish(emp);
}
void QNode::on_OM_z_D2000_clicked(bool check) {
  std_msgs::Empty emp;
  stamper_nikon_cmd_z_d2000.publish(emp);
}

void QNode::on_OM_z_Stamp_clicked(bool check){
  std_msgs::UInt32 pos;
  pos.data = 47680;
  stamper_nikon_cmd_z_abs.publish(pos);
}

void QNode::on_OM_z_AddrFlake_clicked(bool check){
  std_msgs::UInt32 pos;
  pos.data = 39280;
  stamper_nikon_cmd_z_abs.publish(pos);
}

void QNode::on_Sample_Theta_Center_clicked(bool check) {
  geometry_msgs::Point pnt;
  pnt.x = 0;
  pnt.y = 0;
  stamper_sample_theta_cmd_abs.publish(pnt);
}

void QNode::on_Light_Off_Button_clicked(bool check) {
  std_msgs::Byte emp;
  stamper_nic_100a_cmd_off.publish(emp);
}

void QNode::on_Light_On_Button_clicked(bool check) {
  std_msgs::Byte emp;
  stamper_nic_100a_cmd_on.publish(emp);
}

void QNode::on_Light_Inten_valueChanged() {
  std_msgs::UInt8 inten;
  inten.data = spinBox_Light_Inten->value();
  stamper_nic_100a_cmd_inten.publish(inten);
}

void QNode::on_Sample_XY_Spd_valueChanged() {
  stamper_sample_xy::velocity vel;
  vel.velocity_low = spinBox_Sample_XY_Spd->value();
  vel.velocity_high = spinBox_Sample_XY_Spd->value();
  vel.accl = std::min(50, spinBox_Sample_XY_Spd->value());
  stamper_sample_xy_cmd_vel.publish(vel);
}

void QNode::on_Sample_Z_Spd_valueChanged() {
  stamper_sample_z::velocity vel;
  vel.velocity_low = spinBox_Sample_Z_Spd->value();
  vel.velocity_high = 15000;
  vel.accl = std::min(50, spinBox_Sample_Z_Spd->value());
  stamper_sample_z_cmd_vel.publish(vel);
}

void QNode::on_Sample_Theta_Spd_valueChanged() {
  stamper_sample_theta::velocity vel;
  vel.velocity_low = spinBox_Sample_Theta_Spd->value();
  vel.velocity_high = spinBox_Sample_Theta_Spd->value();
  vel.accl = std::min(50, spinBox_Sample_Theta_Spd->value());
  stamper_sample_theta_cmd_vel.publish(vel);
}

void QNode::on_OM_XY_Spd_valueChanged() {
  stamper_om_xy::velocity vel;
  vel.velocity_low = spinBox_OM_XY_Spd->value();
  vel.velocity_high = 15000;
  vel.accl = std::min(50, spinBox_OM_XY_Spd->value());
  stamper_om_xy_cmd_vel.publish(vel);
}

void QNode::on_Stamp_Taihi_clicked(bool check) {
  stamper_om_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel);
  stamper_sample_xy_cmd_vel.publish(vel);

  geometry_msgs::Point pnt;
  pnt.x = 100000;
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);

  pnt.x = -100000;
  pnt.y = 0;
  stamper_sample_xy_cmd_stp.publish(pnt);
}

void QNode::on_Stamp_Fukki_clicked(bool check) {
  stamper_om_xy::velocity vel_om;
  vel_om.velocity_low = 1000;
  vel_om.velocity_high = 15000;
  vel_om.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel_om);

  stamper_sample_xy::velocity vel_sample;
  vel_sample.velocity_low = 1000;
  vel_sample.velocity_high = 15000;
  vel_sample.accl = 100;
  stamper_sample_xy_cmd_vel.publish(vel_sample);

  geometry_msgs::Point pnt;
  pnt.x = -100000;
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);

  pnt.x = 100000;
  pnt.y = 0;
  stamper_sample_xy_cmd_stp.publish(pnt);
}

void QNode::on_Alignment_Pos_clicked(bool check) {
  int pos_sample_xy_x, pos_sample_xy_y;
  pos_sample_xy_x = 750000;
  pos_sample_xy_y = 650000;
  std_msgs::Empty emp;
  keyence_om_back_publisher.publish(emp);

  stamper_sample_xy::velocity vel_sample_xy;
  vel_sample_xy.velocity_low = 1000;
  vel_sample_xy.velocity_high = 15000;
  vel_sample_xy.accl = 100;
  stamper_sample_xy_cmd_vel.publish(vel_sample_xy);

  stamper_om_xy::velocity vel_om_xy;
  vel_om_xy.velocity_low = 1000;
  vel_om_xy.velocity_high = 15000;
  vel_om_xy.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel_om_xy);

  stamper_sample_z::velocity vel_sample_z;
  vel_sample_z.velocity_low = 1000;
  vel_sample_z.velocity_high = 15000;
  vel_sample_z.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel_sample_z);

  ros::Duration(0.5).sleep();
  geometry_msgs::Point pnt;

  pnt.x = 5000;
  pnt.y = 0;
  stamper_sample_z_cmd_abs.publish(pnt);

  std_srvs::Empty emp_srv;
  stamper_sample_z_wait_for_stop_client.call(emp_srv);

  stamper_sample_z::GetCurrentPos pos;
  if (!stamper_sample_z_get_current_pos_client.call(pos)) {
    ROS_ERROR("Stamper Z stage Communication Error");
    return;
  }
  if (!(pos.response.pos < 6000)) {
    ROS_ERROR("Z stage not located at home position");
    return;
  }

  QMessageBox msgBox;
  msgBox.setText(tr("Confirm Z stage position"));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Ok) {
    keyence_mask_fwd_publisher.publish(emp);
  } else {
    return;
  }

  ros::Duration(25).sleep();

  keyence_om_fwd_publisher.publish(emp);

  vel_sample_z.velocity_low = 1000;
  vel_sample_z.velocity_high = 15000;
  vel_sample_z.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel_sample_z);

  pnt.x = pos_sample_xy_x;
  pnt.y = pos_sample_xy_y;
  stamper_sample_xy_cmd_abs.publish(pnt);

  pnt.x = 585000;
  pnt.y = 0;
  stamper_sample_z_cmd_abs.publish(pnt);
  stamper_sample_z_wait_for_stop_client.call(emp_srv);
}

void QNode::on_Exchange_Pos_clicked(bool check) {
  std_msgs::Empty emp;
  keyence_om_back_publisher.publish(emp);

  stamper_sample_xy::velocity vel_sample_xy;
  vel_sample_xy.velocity_low = 1000;
  vel_sample_xy.velocity_high = 15000;
  vel_sample_xy.accl = 100;
  stamper_sample_xy_cmd_vel.publish(vel_sample_xy);

  stamper_om_xy::velocity vel_om_xy;
  vel_om_xy.velocity_low = 1000;
  vel_om_xy.velocity_high = 15000;
  vel_om_xy.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel_om_xy);

  stamper_sample_z::velocity vel_sample_z;
  vel_sample_z.velocity_low = 1000;
  vel_sample_z.velocity_high = 15000;
  vel_sample_z.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel_sample_z);

  ros::Duration(0.5).sleep();

  stamper_sample_xy::GetCurrentPos pos_sample_xy;
  stamper_om_xy::GetCurrentPos pos_om_xy;

  stamper_sample_xy_get_current_pos_client.call(pos_sample_xy);
  stamper_om_xy_get_currrent_pos_client.call(pos_om_xy);

  ros::param::set("pos_sample_xy_x",
                  static_cast<int>(pos_sample_xy.response.pos_x));
  ros::param::set("pos_sample_xy_y",
                  static_cast<int>(pos_sample_xy.response.pos_y));
  ros::param::set("pos_om_xy_x", static_cast<int>(pos_om_xy.response.pos_x));
  ros::param::set("pos_om_xy_y", static_cast<int>(pos_om_xy.response.pos_y));

  geometry_msgs::Point pnt;

  pnt.x = 1000000;
  pnt.y = 5000;
  stamper_sample_xy_cmd_abs.publish(pnt);

  pnt.x = 1400000;
  pnt.y = 5000;
  // stamper_om_xy_cmd_abs.publish(pnt);

  pnt.x = 5000;
  pnt.y = 0;
  stamper_sample_z_cmd_abs.publish(pnt);

  pnt.x = 0;
  pnt.y = 0;
  stamper_sample_theta_cmd_abs.publish(pnt);

  std_srvs::Empty emp_srv;
  stamper_sample_z_wait_for_stop_client.call(emp_srv);

  stamper_sample_z::GetCurrentPos pos;

  if (!stamper_sample_z_get_current_pos_client.call(pos)) {
    ROS_ERROR("Stamper Z stage Communication Error");
    return;
  }
  if (!(pos.response.pos < 10000)) {
    ROS_ERROR("Z stage not located at home position");
    return;
  }

  QMessageBox msgBox;
  msgBox.setText(tr("Confirm Z stage position"));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Ok) {
    std_msgs::Empty emp;
    keyence_mask_back_publisher.publish(emp);
  }
  ros::Duration(0.1).sleep();
  keyence_sample_vacuum_off_publisher.publish(emp);
  pushButton_Sample_Vacuum_On->setChecked(false);
}

}  // namespace tdmms_stamper_teleop
