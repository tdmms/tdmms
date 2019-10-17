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
#include <sstream>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <adm2/velocity.h>
#include <QSound>
#include "../include/tdmms_finder_teleop/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace tdmms_finder_teleop {

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
  ros::init(init_argc, init_argv, "tdmms_finder_teleop");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;

  nic_100a_on_publisher = n.advertise<std_msgs::Byte>("/nic_100a/cmd_on", 1);
  nic_100a_off_publisher = n.advertise<std_msgs::Byte>("/nic_100a/cmd_off", 1);
  nic_100a_inten_publisher =
      n.advertise<std_msgs::UInt8>("/nic_100a/cmd_inten", 1);
  lv_ncnt_n_publisher =
      n.advertise<std_msgs::UInt8>("/lv_ncnt_n_master/cmd_set", 1);

  adm2_step_publisher =
      n.advertise<geometry_msgs::Point>("/adm2_master/cmd_stp_move", 1);

  adm2_vel_publisher = n.advertise<adm2::velocity>("/adm2_master/cmd_vel", 1);
  adm2_abs_publisher =
      n.advertise<geometry_msgs::Point>("/adm2_master/cmd_abs_move", 1);

  adm2_home_publisher =
      n.advertise<std_msgs::Empty>("/adm2_master/cmd_init", 1);

  adm2_jog_publisher =
      n.advertise<geometry_msgs::Point>("/adm2_master/cmd_jog_move", 1);
  adm2_stop_publisher =
      n.advertise<std_msgs::Empty>("/adm2_master/cmd_stop", 1);
  afc_5_publisher = n.advertise<std_msgs::UInt8>("/afc_5_master/cmd_chx", 1);
  afc_5_publisher_sc0 =
      n.advertise<std_msgs::Empty>("/afc_5_master/cmd_sc0", 1);
  ace2000_exp_publisher =
      n.advertise<std_msgs::UInt32>("/ace2000_master/cmd_set_exp", 1);
  bsc201_abs_publisher =
      n.advertise<std_msgs::UInt8>("/bsc201_master/cmd_set", 1);
  bsc201_home_publisher =
      n.advertise<std_msgs::Empty>("/bsc201_master/cmd_home", 1);
  joy_subscriber = n.subscribe("joy", 1, &QNode::joyCallback, this);
  adm2_subscriber =
      n.subscribe("/adm2_master/status", 10, &QNode::adm2Callback, this);

  revolver_pos = 0;
  jog_moving = false;
  start();
  adm2::velocity vel;

  vel.velocity_low = Vel_Low_SpinBox->value();
  vel.velocity_high = Vel_High_SpinBox->value();
  vel.accl = Accl_SpinBox->value();
  adm2_vel_publisher.publish(vel);

  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "tdmms_finder_teleop");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;
  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ros::spin();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to
                         // roslaunch)
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

void QNode::adm2Callback(const geometry_msgs::Point &currpoint) {
  Pos_X_SpinBox->setValue(currpoint.x);
  Pos_Y_SpinBox->setValue(currpoint.y);
}

void QNode::joyCallback(const sensor_msgs::Joy &joy_msg) {
  if (joy_msg.buttons[11] == 1) {  //
    std_msgs::Byte b;
    nic_100a_on_publisher.publish(b);
    ROS_INFO("Light ON");
  } else if (joy_msg.buttons[10] == 1) {  //
    std_msgs::Byte b;
    nic_100a_off_publisher.publish(b);
    ROS_INFO("Light OFF");
  } else if (joy_msg.buttons[0] == 1) {  // SC0
    std_msgs::Empty emp;
    afc_5_publisher_sc0.publish(emp);
    ROS_INFO("Execute AF");
  } else if (joy_msg.buttons[1] == 1) {
    std_msgs::Empty emp;
    ROS_INFO("STOP Requested");
    adm2_stop_publisher.publish(emp);
  } else if (joy_msg.buttons[4] == 1) {  //
    std_msgs::UInt8 rev;
    revolver_pos = (revolver_pos - 1) % 5;
    rev.data = revolver_pos + 1;
    lv_ncnt_n_publisher.publish(rev);
    afc_5_publisher.publish(rev);
    Revolver_Pos_SpinBox->setValue(rev.data);

    ROS_INFO("Changing Revolver Position to: %d", rev.data);
    int inten_tuple[] = {70, 80, 130, 220, 250};
    Light_Intensity_SpinBox->setValue(inten_tuple[revolver_pos]);
  } else if (joy_msg.buttons[5] == 1) {
    std_msgs::UInt8 rev;
    revolver_pos = (revolver_pos + 1) % 5;
    rev.data = revolver_pos + 1;
    lv_ncnt_n_publisher.publish(rev);
    afc_5_publisher.publish(rev);
    Revolver_Pos_SpinBox->setValue(rev.data);
    ROS_INFO("Changing Revolver Position to: %d", rev.data);
    int inten_tuple[] = {70, 80, 130, 220, 250};
    Light_Intensity_SpinBox->setValue(inten_tuple[revolver_pos]);
  } else if (joy_msg.buttons[6] == 1) {
    std_msgs::UInt8 fil;
    filter_pos++;
    filter_pos = filter_pos % 6;
    fil.data = filter_pos + 1;
    bsc201_abs_publisher.publish(fil);
    Filter_Pos_SpinBox->setValue(fil.data);
    ROS_INFO("Changing Filter Position to: %d", filter_pos);
  } else if (joy_msg.buttons[7] == 1) {
    std_msgs::UInt8 fil;
    if (filter_pos == 0) {
      filter_pos += 5;
    } else {
      filter_pos--;
    }
    filter_pos = filter_pos % 6;
    fil.data = filter_pos + 1;
    bsc201_abs_publisher.publish(fil);
    Filter_Pos_SpinBox->setValue(fil.data);
    ROS_INFO("Changing Filter Position to: %d", filter_pos);
  } else if (joy_msg.axes[5] > 0 && joy_msg.buttons[2] == 0) {
    geometry_msgs::Point stp;
    stp.x = -Step_SpinBox->value();
    stp.y = 0;
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move X by: %d", (int)stp.x);
  } else if (joy_msg.axes[5] < 0 && joy_msg.buttons[2] == 0) {
    geometry_msgs::Point stp;
    stp.x = Step_SpinBox->value();
    stp.y = 0;
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move X by: %d", (int)stp.x);
  } else if (joy_msg.axes[6] > 0 && joy_msg.buttons[2] == 0) {
    geometry_msgs::Point stp;
    stp.x = 0;
    stp.y = Step_SpinBox->value();
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move Y by: %d", (int)stp.y);
  } else if (joy_msg.axes[6] < 0 && joy_msg.buttons[2] == 0) {
    geometry_msgs::Point stp;
    stp.x = 0;
    stp.y = -Step_SpinBox->value();
    adm2_step_publisher.publish(stp);
    ROS_INFO("Step Move Y by: %d", (int)stp.y);
  } else if ((joy_msg.buttons[2] == 1) &&
             ((joy_msg.axes[5] != 0) || (joy_msg.axes[6] != 0))) {
    jog_moving = true;
    geometry_msgs::Point stp;
    stp.x = -joy_msg.axes[5];
    stp.y = joy_msg.axes[6];
    adm2_jog_publisher.publish(stp);
    ROS_INFO("JOG Move Started");
  } else if ((jog_moving == true) && (joy_msg.axes[5] == 0) &&
             (joy_msg.axes[6] == 0)) {
    jog_moving = false;
    std_msgs::Empty emp;
    adm2_stop_publisher.publish(emp);
    ROS_INFO("JOG Move Stopped");
  }
}

void QNode::on_Light_Intensity_SpinBox_valueChanged() {
  std_msgs::UInt8 inten;
  inten.data = Light_Intensity_SpinBox->value();
  nic_100a_inten_publisher.publish(inten);
  ROS_INFO("Changing Light Intensity to: %d", (int)inten.data);
}

void QNode::on_Step_SpinBox_valueChanged() {}

void QNode::on_Vel_SpinBox_valueChanged() {
  adm2::velocity vel;
  vel.velocity_low = Vel_Low_SpinBox->value();
  vel.velocity_high = Vel_High_SpinBox->value();
  vel.accl = Accl_SpinBox->value();
  adm2_vel_publisher.publish(vel);
  ROS_INFO("Setting Velocity low, high, acceleration: %d, %d, %d",
           vel.velocity_low, vel.velocity_high, vel.accl);
}

void QNode::on_Exposure_SpinBox_valueChanged() {
  std_msgs::UInt32 exp;
  exp.data = Exposure_SpinBox->value();
  ace2000_exp_publisher.publish(exp);
  ROS_INFO("Setting Exposure Time (us): %d", (int)exp.data);
}

void QNode::on_Light_ON_Button_clicked(bool check) {
  std_msgs::Byte b;
  nic_100a_on_publisher.publish(b);
  ROS_INFO("Light ON");
}

void QNode::on_Light_OFF_Button_clicked(bool check) {
  std_msgs::Byte b;
  nic_100a_off_publisher.publish(b);
  ROS_INFO("Light OFF");
}

void QNode::on_Intensity_Descend_Button_clicked(bool check) {
  std_msgs::UInt8 inten;
  inten.data = Light_Intensity_SpinBox->value();
  while(inten.data > 0) {
    nic_100a_inten_publisher.publish(inten);
    inten.data = inten.data -1;
    ros::Duration(1.0).sleep();
    ROS_INFO("Changing Light Intensity to: %d", (int)inten.data);
  }
}

void QNode::on_Move_01_clicked(bool check) { MoveTo(1); }

void QNode::on_Move_02_clicked(bool check) { MoveTo(2); }

void QNode::on_Move_03_clicked(bool check) { MoveTo(3); }

void QNode::on_Move_04_clicked(bool check) { MoveTo(4); }

void QNode::on_Move_05_clicked(bool check) { MoveTo(5); }

void QNode::on_Move_06_clicked(bool check) { MoveTo(6); }

void QNode::on_Move_07_clicked(bool check) { MoveTo(7); }

void QNode::on_Move_08_clicked(bool check) { MoveTo(8); }

void QNode::on_Move_09_clicked(bool check) { MoveTo(9); }

void QNode::on_Move_10_clicked(bool check) { MoveTo(10); }

void QNode::on_Move_11_clicked(bool check) { MoveTo(11); }

void QNode::on_Move_12_clicked(bool check) { MoveTo(12); }

void QNode::on_Move_13_clicked(bool check) { MoveTo(13); }

void QNode::on_Move_14_clicked(bool check) { MoveTo(14); }

void QNode::on_Move_15_clicked(bool check) { MoveTo(15); }

void QNode::on_Move_16_clicked(bool check) { MoveTo(16); }

void QNode::on_Move_17_clicked(bool check) { MoveTo(17); }

void QNode::on_Move_18_clicked(bool check) { MoveTo(18); }

void QNode::on_Move_19_clicked(bool check) { MoveTo(19); }

void QNode::on_Move_20_clicked(bool check) { MoveTo(20); }

void QNode::on_Move_21_clicked(bool check) { MoveTo(21); }

void QNode::on_Move_22_clicked(bool check) { MoveTo(22); }

void QNode::on_Move_23_clicked(bool check) { MoveTo(23); }

void QNode::on_Move_24_clicked(bool check) { MoveTo(24); }

void QNode::on_Move_25_clicked(bool check) { MoveTo(25); }

void QNode::on_Move_26_clicked(bool check) { MoveTo(26); }

void QNode::on_Move_27_clicked(bool check) { MoveTo(27); }

void QNode::on_Move_28_clicked(bool check) { MoveTo(28); }

void QNode::on_Move_29_clicked(bool check) { MoveTo(29); }

void QNode::on_Move_30_clicked(bool check) { MoveTo(30); }

void QNode::on_Move_31_clicked(bool check) { MoveTo(31); }

void QNode::on_Move_32_clicked(bool check) { MoveTo(32); }

void QNode::on_Move_33_clicked(bool check) { MoveTo(33); }

void QNode::on_Move_34_clicked(bool check) { MoveTo(34); }

void QNode::on_Move_35_clicked(bool check) { MoveTo(35); }

void QNode::on_Move_36_clicked(bool check) { MoveTo(36); }

void QNode::on_HP_clicked(bool check) {
  std_msgs::Empty emp;
  adm2_home_publisher.publish(emp);
}

void QNode::MoveTo(int pocketNo) {
  if (pocketNo < 1 || pocketNo > 36) {
    ROS_ERROR("Pocket No out of range");
    return;
  }

  pocketNo--;
  int index_row = pocketNo / 6;
  int index_column = pocketNo % 6;

  const int origin_row = 21520;
  const int origin_column = -21000;
  const int c_offset = 2500;
  const int dbp = 7100;

  int offset_ind_row, offset_ind_column;
  offset_ind_row = 0;
  offset_ind_column = 0;

  if (radioButton_LeftTop->isChecked()) {
    offset_ind_row = -1;
    offset_ind_column = -1;
  }
  if (radioButton_CenterTop->isChecked()) {
    offset_ind_row = -1;
    offset_ind_column = 0;
  }
  if (radioButton_RightTop->isChecked()) {
    offset_ind_row = -1;
    offset_ind_column = 1;
  }
  if (radioButton_LeftCenter->isChecked()) {
    offset_ind_row = 0;
    offset_ind_column = -1;
  }
  if (radioButton_Center->isChecked()) {
    offset_ind_row = 0;
    offset_ind_column = 0;
  }
  if (radioButton_RightCenter->isChecked()) {
    offset_ind_row = 0;
    offset_ind_column = 1;
  }
  if (radioButton_LeftBottom->isChecked()) {
    offset_ind_row = 1;
    offset_ind_column = -1;
  }
  if (radioButton_CenterBottom->isChecked()) {
    offset_ind_row = 1;
    offset_ind_column = 0;
  }
  if (radioButton_RightBottom->isChecked()) {
    offset_ind_row = 1;
    offset_ind_column = 1;
  }

  geometry_msgs::Point point;
  point.y = origin_row - dbp * index_row - offset_ind_row * c_offset;
  point.x = origin_column + dbp * index_column + offset_ind_column * c_offset;

  adm2_abs_publisher.publish(point);
}

}  // namespace tdmms_finder_teleop
