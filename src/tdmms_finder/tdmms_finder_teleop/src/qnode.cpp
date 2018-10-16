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
#include <sstream>
#include "../include/tdmms_finder_teleop/qnode.hpp"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <adm2/velocity.h>
#include <QSound>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace tdmms_finder_teleop {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{
}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"tdmms_finder_teleop");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  
  nic_100a_on_publisher = n.advertise<std_msgs::Byte>("/nic_100a/cmd_on",1);
  nic_100a_off_publisher = n.advertise<std_msgs::Byte>("/nic_100a/cmd_off",1);
  nic_100a_inten_publisher = n.advertise<std_msgs::UInt8>("/nic_100a/cmd_inten",1);
  lv_ncnt_n_publisher = n.advertise<std_msgs::UInt8>("/lv_ncnt_n_master/cmd_set",1);
  adm2_step_publisher = n.advertise<geometry_msgs::Point>("/adm2_master/cmd_stp_move",1);
  adm2_vel_publisher = n.advertise<adm2::velocity>("/adm2_master/cmd_vel",1);
  adm2_jog_publisher = n.advertise<geometry_msgs::Point>("/adm2_master/cmd_jog_move",1);
  adm2_stop_publisher = n.advertise<std_msgs::Empty>("/adm2_master/cmd_stop",1);
  afc_5_publisher = n.advertise<std_msgs::UInt8>("/afc_5_master/cmd_chx",1);
  afc_5_publisher_sc0 = n.advertise<std_msgs::Empty>("/afc_5_master/cmd_sc0",1);
  ace2000_exp_publisher = n.advertise<std_msgs::UInt32>("/ace2000_master/cmd_set_exp",1);
  bsc201_abs_publisher = n.advertise<std_msgs::UInt8>("/bsc201_master/cmd_set",1);
  bsc201_home_publisher = n.advertise<std_msgs::Empty>("/bsc201_master/cmd_home",1);
  joy_subscriber = n.subscribe("joy", 1, &QNode::joyCallback,this);
  adm2_subscriber = n.subscribe("/adm2_master/status", 10, &QNode::adm2Callback, this);

  revolver_pos = 0;
  jog_moving= false;
  start();
  adm2::velocity vel;

  vel.velocity_low = Vel_Low_SpinBox->value();
  vel.velocity_high = Vel_High_SpinBox->value();
  vel.accl = Accl_SpinBox->value();
  adm2_vel_publisher.publish(vel);
  
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"tdmms_finder_teleop");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  int count = 0;
  while ( ros::ok() ) {
    std_msgs::String msg;
    std::stringstream ss;
    ros::spin();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
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
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::adm2Callback(const geometry_msgs::Point &currpoint)
{
  Pos_X_SpinBox->setValue(currpoint.x);
  Pos_Y_SpinBox->setValue(currpoint.y);
}

void QNode::joyCallback(const sensor_msgs::Joy &joy_msg){
  if(joy_msg.buttons[11] == 1){         //
    std_msgs::Byte b;
    nic_100a_on_publisher.publish(b);
    ROS_INFO("Light ON");
  }else if (joy_msg.buttons[10] == 1){  //
    std_msgs::Byte b;
    nic_100a_off_publisher.publish(b);
    ROS_INFO("Light OFF");
  }else if (joy_msg.buttons[0] == 1){   //SC0
    std_msgs::Empty emp;
    afc_5_publisher_sc0.publish(emp);
    ROS_INFO("Execute AF");
  }else if (joy_msg.buttons[1] == 1){
    std_msgs::Empty emp;
    ROS_INFO("STOP Requested");
    adm2_stop_publisher.publish(emp);
  }else if (joy_msg.buttons[4] == 1){    //
    std_msgs::UInt8 rev;
    revolver_pos = (revolver_pos -1 )%5;
    rev.data = revolver_pos+1;
    lv_ncnt_n_publisher.publish(rev);
    afc_5_publisher.publish(rev);
    Revolver_Pos_SpinBox->setValue(rev.data);

    ROS_INFO("Changing Revolver Position to: %d", rev.data);
    int inten_tuple[] = {70, 80,130 ,220, 250};
    Light_Intensity_SpinBox->setValue(inten_tuple[revolver_pos]);
  }else if (joy_msg.buttons[5] == 1){
    std_msgs::UInt8 rev;
    revolver_pos = (revolver_pos +1 )%5;
    rev.data = revolver_pos+1;
    lv_ncnt_n_publisher.publish(rev);
    afc_5_publisher.publish(rev);
    Revolver_Pos_SpinBox->setValue(rev.data);
    ROS_INFO("Changing Revolver Position to: %d", rev.data);
    int inten_tuple[] = {70, 80,130 ,220, 250};
    Light_Intensity_SpinBox->setValue(inten_tuple[revolver_pos]);
  }else if(joy_msg.buttons[6] == 1){
    std_msgs::UInt8 fil;
    filter_pos++;
    filter_pos = filter_pos % 6;
    fil.data = filter_pos + 1;
    bsc201_abs_publisher.publish(fil);
    Filter_Pos_SpinBox->setValue(fil.data);
    ROS_INFO("Changing Filter Position to: %d", filter_pos);
  }else if(joy_msg.buttons[7] == 1){
    std_msgs::UInt8 fil;
    if(filter_pos== 0){
      filter_pos += 5;
    }else{
      filter_pos --;
    }
    filter_pos = filter_pos % 6;
    fil.data = filter_pos + 1;
    bsc201_abs_publisher.publish(fil);
    Filter_Pos_SpinBox->setValue(fil.data);
    ROS_INFO("Changing Filter Position to: %d", filter_pos);
  }else if (joy_msg.axes[5] > 0 && joy_msg.buttons[2] == 0){
    geometry_msgs::Point stp;
    stp.x = -Step_SpinBox->value();
    stp.y = 0;
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move X by: %d", (int)stp.x);
  }else if (joy_msg.axes[5] < 0 && joy_msg.buttons[2] == 0){
    geometry_msgs::Point stp;
    stp.x = Step_SpinBox->value();
    stp.y = 0;
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move X by: %d", (int)stp.x);
  }else if (joy_msg.axes[6] > 0 && joy_msg.buttons[2] == 0){
    geometry_msgs::Point stp;
    stp.x = 0;
    stp.y = Step_SpinBox->value();
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move Y by: %d", (int)stp.y);
  }else if (joy_msg.axes[6] < 0 && joy_msg.buttons[2] == 0){
    geometry_msgs::Point stp;
    stp.x = 0;
    stp.y = -Step_SpinBox->value();
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move Y by: %d", (int)stp.y);
  }else if ((joy_msg.buttons[2] == 1) && ((joy_msg.axes[5] != 0 )||(joy_msg.axes[6]!=0))){
    jog_moving=true;
    geometry_msgs::Point stp;
    stp.x = -joy_msg.axes[5];
    stp.y = joy_msg.axes[6];
    adm2_jog_publisher.publish(stp);
    ROS_INFO("JOG Move Started");
  }else if ((jog_moving==true)
            && (joy_msg.axes[5] == 0)
            && (joy_msg.axes[6] == 0)){
    jog_moving=false;
    std_msgs::Empty emp;
    adm2_stop_publisher.publish(emp);
    ROS_INFO("JOG Move Stopped");
  }
}

void QNode::on_Light_Intensity_SpinBox_valueChanged(){
  std_msgs::UInt8 inten;
  inten.data = Light_Intensity_SpinBox->value();
  nic_100a_inten_publisher.publish(inten);
  ROS_INFO("Changing Light Intensity to: %d",(int) inten.data);
}

void QNode::on_Step_SpinBox_valueChanged(){
}

void QNode::on_Vel_SpinBox_valueChanged(){
  adm2::velocity vel;
  vel.velocity_low = Vel_Low_SpinBox->value();
  vel.velocity_high = Vel_High_SpinBox->value();
  vel.accl = Accl_SpinBox->value();
  adm2_vel_publisher.publish(vel);
  ROS_INFO("Setting Velocity low, high, acceleration: %d, %d, %d", vel.velocity_low, vel.velocity_high, vel.accl);
}

void QNode::on_Exposure_SpinBox_valueChanged(){
  std_msgs::UInt32 exp;
  exp.data = Exposure_SpinBox->value();
  ace2000_exp_publisher.publish(exp);
  ROS_INFO("Setting Exposure Time (us): %d", (int) exp.data);
}

void QNode::on_Light_ON_Button_clicked(bool check)
{
  std_msgs::Byte b;
  nic_100a_on_publisher.publish(b);
  ROS_INFO("Light ON");
}

void QNode::on_Light_OFF_Button_clicked(bool check)
{
  std_msgs::Byte b;
  nic_100a_off_publisher.publish(b);
  ROS_INFO("Light OFF");
}

}  // namespace tdmms_finder_teleop

