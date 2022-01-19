/////////////////////////////////////
//  Copyright 2016.05 by S. Masubuchi
//  2DMMS Autostamp Node
/////////////////////////////////////

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <ros/network.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_goal_state.h>
#include <stamper_sample_z/velocity.h>
#include <stamper_sample_z/GetCurrentPos.h>
#include <stamper_sample_xy/velocity.h>
#include <stamper_sample_theta/velocity.h>
#include <stamper_sample_xy/GetCurrentPos.h>
#include <stamper_om_xy/velocity.h>
#include <stamper_om_xy/GetCurrentPos.h>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/Empty.h>

#include <tdmms_autostamp_flake_positioner_action_fast/PositionFlakeFastAction.h>
#include <tdmms_autostamp_flake_positioner_action_fast2/PositionFlakeFast2Action.h>
#include <tdmms_autostamp_flake_positioner_action/PositionFlakeAction.h>
#include <tdmms_autostamp_flake_aligner_xy10x_action/AlignFlakeXY10xAction.h>
#include <tdmms_autostamp_flake_aligner_xytheta5x_action/AlignFlakeXYTheta5xAction.h>
#include <tdmms_autostamp_chip_transfer_action/TransferChipAction.h>
#include <tdmms_autostamp_stamp_action/StampAction.h>
#include <tdmms_autostamp_autofocus_action/AutoFocusAction.h>
#include <tdmms_autostamp_flake_aligner_ncc_action/AlignFlakeNCCAction.h>
#include <tdmms_autostamp_camera_streamer/CaptureImage.h>
#include <tdmms_autostamp_camera_streamer/CaptureImageTimestamp.h>
#include <halcon_bridge/halcon_bridge.h>

#include <QtSql>
#include <QtGui>
#include <QWidget>
#include <QtGui/QMainWindow>
#include <QMessageBox>
#include <QFileDialog>

#include <string>
#include <map>
#include <sstream>
#include "../include/tdmms_autostamp_master/qnode.hpp"
#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace tdmms_autostamp_master {
using namespace HalconCpp;
using namespace Qt;

/*****************************************************************************
 * Implementation
 *****************************************************************************/

QNode::QNode(int argc, char **argv)
    : init_argc(argc),
      init_argv(argv),
      ac_flake_positioner_fast("/tdmms_autostamp_flake_positioner_action_fast",
                               true),
      ac_flake_positioner_fast2("/tdmms_autostamp_flake_positioner_action_fast2",
                               true),
      ac_flake_positioner_fast3("/tdmms_autostamp_flake_positioner_action_fast3",
                               true),
      ac_flake_positioner("/tdmms_autostamp_flake_positioner_action", true),
      ac_chip_transfer("/tdmms_autostamp_chip_transfer_action", true),
      ac_flake_aligner_xy10x("/tdmms_autostamp_flake_aligner_xy10x_action",
                             true),
      ac_flake_aligner_xy10x2("/tdmms_autostamp_flake_aligner_xy10x2_action",
                             true),
      ac_flake_aligner_xytheta5x(
          "/tdmms_autostamp_flake_aligner_xytheta5x_action", true),
      ac_flake_aligner_xytheta5x2(
          "/tdmms_autostamp_flake_aligner_xytheta5x2_action", true),
      ac_flake_aligner_xytheta5x3(
          "/tdmms_autostamp_flake_aligner_xytheta5x3_action", true),
      ac_flake_aligner_xy10x_fine(
          "/tdmms_autostamp_flake_aligner_xy10x_fine_action", true),
      ac_stamp("/tdmms_autostamp_stamp_action", true),
      ac_autofocus("/tdmms_autostamp_autofocus_action", true),
      ac_flake_aligner_ncc("/tdmms_autostamp_flake_aligner_ncc_action", true) {
  /////////////////////////////////////////////////////////////
  //// Waiting for action server to start
  /////////////////////////////////////////////////////////////
  ROS_INFO("Waiting for action server to start.");

  ac_flake_positioner_fast.waitForServer();
  ac_flake_positioner_fast2.waitForServer();
  ac_flake_positioner_fast3.waitForServer();
  ac_flake_positioner.waitForServer();
  ac_chip_transfer.waitForServer();
  ac_flake_aligner_xy10x.waitForServer();
  ac_flake_aligner_xy10x2.waitForServer();
  ac_flake_aligner_xytheta5x.waitForServer();
  ac_flake_aligner_xytheta5x2.waitForServer();
  ac_flake_aligner_xytheta5x3.waitForServer();
  ac_flake_aligner_xy10x_fine.waitForServer();
  ac_stamp.waitForServer();
  ac_autofocus.waitForServer();
  ac_flake_aligner_ncc.waitForServer();
  ROS_INFO("Action server started");
  skip_requested = false;
  fileVer = 1;
}

QNode::~QNode() {
  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  if (!ros::master::check()) {
    return false;
  }
  ros::start();
  ros::NodeHandle n;
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  if (!ros::master::check()) {
    return false;
  }
  ros::start();
  ros::NodeHandle n;

  /////////////////////////////////////
  // ROS communications
  /////////////////////////////////////
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  designer_filename_subscriber = n.subscribe(
      "/designer/filename", 10, &QNode::filename_updated_Callback, this);
  transfer_linear_cmd_abs_move =
      n.serviceClient<transfer_linear_yamaha::AbsMove>(
          "/transfer_linear_yamaha_master/cmd_abs_move");
  stamper_sample_z_cmd_vel = n.advertise<stamper_sample_z::velocity>(
      "/stamper_sample_z_master/cmd_vel", 1);
  stamper_sample_z_cmd_abs = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_z_master/cmd_abs_move", 1);
  stamper_sample_z_wait_for_stop_client = n.serviceClient<std_srvs::Empty>(
      "/stamper_sample_z_master/wait_for_stop");
  stamper_sample_z_wait_for_stop_client = n.serviceClient<std_srvs::Empty>(
      "/stamper_sample_z_master/wait_for_stop");
  stamper_sample_xy_wait_for_stop_client = n.serviceClient<std_srvs::Empty>(
      "/stamper_sample_xy_master/wait_for_stop");
  stamper_om_xy_wait_for_stop_client =
      n.serviceClient<std_srvs::Empty>("/stamper_om_xy_master/wait_for_stop");

  stamper_sample_z_get_current_pos_client =
      n.serviceClient<stamper_sample_z::GetCurrentPos>(
          "/stamper_sample_z_master/get_currpos");

  stamper_sample_xy_cmd_abs = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_xy_master/cmd_abs_move", 1);
  stamper_nikon_cmd_z_abs =
      n.advertise<std_msgs::UInt32>("/stamper_nikon_master/cmd_z_abs", 1);
  keyence_sample_vacuum_on_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_vacuum_on", 1);
  keyence_sample_vacuum_off_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_sample_vacuum_off", 1);
  keyence_om_fwd_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_om_fwd", 1);
  keyence_om_back_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_om_back", 1);
  keyence_mask_fwd_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_mask_fwd", 1);
  keyence_mask_back_publisher =
      n.advertise<std_msgs::Empty>("/keyence_master/cmd_mask_back", 1);

  stamper_sample_xy_cmd_vel = n.advertise<stamper_sample_xy::velocity>(
      "/stamper_sample_xy_master/cmd_vel", 1);
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
  stamper_sample_theta_cmd_vel = n.advertise<stamper_sample_theta::velocity>(
      "/stamper_sample_theta_master/cmd_vel", 1);
  stamper_sample_theta_cmd_jog = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_theta_master/cmd_jog_move", 1);
  stamper_sample_theta_cmd_abs = n.advertise<geometry_msgs::Point>(
      "/stamper_sample_theta_master/cmd_abs_move", 1);
  stamper_sample_theta_cmd_stop =
      n.advertise<std_msgs::Empty>("/stamper_sample_theta_master/cmd_stop", 1);

  stamper_om_xy_cmd_vel =
      n.advertise<stamper_om_xy::velocity>("/stamper_om_xy_master/cmd_vel", 1);
  stamper_om_xy_cmd_stp = n.advertise<geometry_msgs::Point>(
      "/stamper_om_xy_master/cmd_stp_move", 1);
  stamper_om_xy_get_currrent_pos_client =
      n.serviceClient<stamper_om_xy::GetCurrentPos>(
          "/stamper_om_xy_master/get_currpos");

  autostamp_camera_streamer_capture =
      n.serviceClient<tdmms_autostamp_camera_streamer::CaptureImage>(
          "/autostamp_camera_streamer/capture");

  autostamp_camera_streamer_capture_timestamp =
      n.serviceClient<tdmms_autostamp_camera_streamer::CaptureImageTimestamp>(
          "/autostamp_camera_streamer/capture_timestamp");

  /*****************************************************************************
   ** Connect to MySQL Database (localhost)
   *****************************************************************************/
  QSqlError err;
  if (QSqlDatabase::drivers().isEmpty()) {
    ROS_ERROR(
        "No database drivers found"
        "This program requires at least one Qt database driver. "
        "Please check the documentation how to build the "
        "Qt SQL plugins.");
  }

  db =
      QSqlDatabase::addDatabase(QString("QMYSQL"), QString("Browser%1").arg(1));
  db.setDatabaseName(QString("mydb"));
  db.setHostName(QString("localhost"));
  db.setPort(3306);
  if (!db.open(QString("root"), QString("mlab2dmms"))) {
    err = db.lastError();
    db = QSqlDatabase();
    QSqlDatabase::removeDatabase(QString("Browser%1").arg(1));
    ROS_ERROR("Error connecting to MySQL Server");
    return false;
  } else {
    ROS_INFO("connected to MYSQL Server");
  }

  /*****************************************************************************
   ** Connect to MySQL Database (AWS Cloud)
   *****************************************************************************/
  //QSqlError err;
  if (QSqlDatabase::drivers().isEmpty()) {
    ROS_ERROR(
        "No database drivers found"
        "This program requires at least one Qt database driver. "
        "Please check the documentation how to build the "
        "Qt SQL plugins.");
  }

  db2 =
      QSqlDatabase::addDatabase(QString("QMYSQL"), QString("Browser%1").arg(2));
  db2.setDatabaseName(QString("2dmms_db"));
  db2.setHostName(QProcessEnvironment::systemEnvironment().value("AWS_DB_URL","default"));
  db2.setPort(3306);
  if (!db2.open(QProcessEnvironment::systemEnvironment().value("AWS_DB_USER","default"),
                QProcessEnvironment::systemEnvironment().value("AWS_DB_PW","default"))) {
    err = db2.lastError();
    db2 = QSqlDatabase();
    QSqlDatabase::removeDatabase(QString("Browser%1").arg(2));
    ROS_ERROR("Error connecting to MySQL Server");
    return false;
  } else {
    ROS_INFO("connected to MYSQL Server");
  }
  
  start();
  return true;
}

void QNode::filename_updated_Callback(const std_msgs::String &fname) {
  ROS_INFO("Update Filename");
  lineEdit_Filename->setText(fname.data.c_str());
}

void QNode::run() {
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
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
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

bool QNode::on_AutoFocus_clicked(bool check) {
  tdmms_autostamp_autofocus_action::AutoFocusGoal goal;
  goal.start.data = spinBox_AutoFocus_Start->value();
  goal.delta_z.data = spinBox_AutoFocus_Step->value();
  goal.stepno.data = spinBox_AutoFocus_Count->value();
  ac_autofocus.sendGoal(goal,
                        boost::bind(&QNode::doneAutoFocusCb, this, _1, _2),
                        actionlib::SimpleActionClient<
                            tdmms_autostamp_autofocus_action::AutoFocusAction>::
                            SimpleActiveCallback(),
                        actionlib::SimpleActionClient<
                            tdmms_autostamp_autofocus_action::AutoFocusAction>::
                            SimpleFeedbackCallback());
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state = ac_autofocus.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_ERROR("Auto Focus Error");
      return false;
    }
  }
}

bool QNode::setFileVer(int _fileVer) {
  fileVer = _fileVer;
  return true;
}

void QNode::doneAutoFocusCb(
    const actionlib::SimpleClientGoalState &state,
    const tdmms_autostamp_autofocus_action::AutoFocusResultConstPtr &result) {
  ROS_INFO("Done AutoFocus: %d", result->focuspos.data);
  spinBox_AutoFocus_Zpos->setValue(result->focuspos.data);
}

bool QNode::on_StartAutostamp_Full_clicked(bool check) {
  int row = tdmtableWidget->currentRow();

  while (row < tdmtableWidget->rowCount()) {
    if (this->on_StartAutostamp_clicked(true)) {
      row++;
      tdmtableWidget->setCurrentCell(row, 0);
      ros::Duration(1).sleep();
    } else {
      return false;
    }
  }
  ROS_INFO("Autostamp_FULL_Succeeded");
  return true;
}

void QNode::on_Stop_clicked(bool check) {
  // if (ac_flake_positioner_fast2.getState() ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_positioner_fast2.cancelGoal();
  ac_flake_positioner_fast3.cancelGoal();
  //skip_requested = false;
  //}

  //  if (ac_flake_aligner_xytheta5x2.getState() ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_aligner_xytheta5x2.cancelGoal();
  ac_flake_aligner_xytheta5x3.cancelGoal();
  //  skip_requested = false;
  //}

  //  if (ac_flake_aligner_xy10x2.getState () ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_aligner_xy10x2.cancelGoal();
  //skip_requested = false;
  //  }

  //  if (ac_flake_positioner.getState() ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_positioner.cancelGoal();
  //  skip_requested = false;
  //  }
  //  if (ac_flake_positioner_fast.getState() ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_positioner_fast.cancelGoal();
  //skip_requested = false;
  //  }
  //if (ac_chip_transfer.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
  ac_chip_transfer.cancelGoal();
  //skip_requested = false;
  //}
  //  if (ac_flake_aligner_xy10x.getState() ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_aligner_xy10x.cancelGoal();
  //skip_requested = false;
  //  }
  //if (ac_flake_aligner_xytheta5x.getState() ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_aligner_xytheta5x.cancelGoal();
  //skip_requested = false;
  //}
  //if (ac_flake_aligner_xy10x_fine.getState() ==
  //  actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_aligner_xy10x_fine.cancelGoal();
  //  skip_requested = false;
  //  }
  //  if (ac_stamp.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
  ac_stamp.cancelGoal();
  //skip_requested = false;
  //  }
  // if (ac_autofocus.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
  //ac_autofocus.cancelGoal();
  skip_requested = false;
  //  }
  //if (ac_flake_aligner_ncc.getState() ==
  //actionlib::SimpleClientGoalState::ACTIVE) {
  ac_flake_aligner_ncc.cancelGoal();
  //skip_requested = false;
  //}
}

bool QNode::on_Skip_clicked(bool check) {
  /*
  if (ac_flake_positioner.getState() ==
      actionlib::SimpleClientGoalState::ACTIVE) {
    ac_flake_positioner.cancelGoal();
    skip_requested = true;
  }
  if (ac_flake_positioner_fast.getState() ==
      actionlib::SimpleClientGoalState::ACTIVE) {
    ac_flake_positioner_fast.cancelGoal();
    skip_requested = true;
  }
  if (ac_chip_transfer.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    ac_chip_transfer.cancelGoal();
    skip_requested = false;
  }
  if (ac_flake_aligner_xy10x.getState() ==
      actionlib::SimpleClientGoalState::ACTIVE) {
    ac_flake_aligner_xy10x.cancelGoal();
    skip_requested = true;
  }
  if (ac_flake_aligner_xytheta5x.getState() ==
      actionlib::SimpleClientGoalState::ACTIVE) {
    ac_flake_aligner_xytheta5x.cancelGoal();
    skip_requested = true;
  }
  if (ac_flake_aligner_xy10x_fine.getState() ==
      actionlib::SimpleClientGoalState::ACTIVE) {
    ac_flake_aligner_xy10x_fine.cancelGoal();
    skip_requested = true;
  }
  if (ac_stamp.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    ac_stamp.cancelGoal();
    skip_requested = true;
  }
  if (ac_autofocus.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    ac_autofocus.cancelGoal();
    skip_requested = true;
  }
  if (ac_flake_aligner_ncc.getState() ==
      actionlib::SimpleClientGoalState::ACTIVE) {
    ac_flake_aligner_ncc.cancelGoal();
    skip_requested = true;
  }
  */
  return true;
}

bool QNode::on_StartAutostamp_clicked(bool check) {
  ///////////////////
  // Load Chip
  //////////////////
  if (!this->on_LoadChip_clicked(true)) {
    ROS_ERROR("LoadChip");
    return false;
  }

  //////////////////
  /// Goto Alignment Position
  //////////////////
  if (!this->on_Alignment_Pos_clicked(true)) {
    ROS_ERROR("AlignmentPos");
    return false;
  }

  ros::Duration(2).sleep();

  /////////////////////
  /// Automated Alignment
  /////////////////////
  if (!this->on_AutoAlign_clicked(true)) {
    ROS_ERROR("Alignment");
    return false;
  }

  QMessageBox msgBox;
  msgBox.setText(tr("Confirm Alignment Position"));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Cancel) {
    return false;
  }

  //////////////////
  // Capture Image
  /////////////////
  tdmms_autostamp_camera_streamer::CaptureImageTimestamp cap_timestamp;
  cap_timestamp.request.folder = static_cast<const char *>(
      lineEdit_picFolder->text().toStdString().c_str());
  autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);

  //////////////////
  //// Stamp Fukki
  //////////////////
  this->on_Stamp_Fukki_clicked(true);

  /////////////////
  ///// Stamp
  ////////////////
  if (!this->on_Stamp_clicked(true)) {
    if (skip_requested) {
      skip_requested = false;
    } else {
      ROS_ERROR("Stamp");
      return false;
    }
  }

  ////////////////
  //// Stamp Taihi
  ////////////////
  this->on_Stamp_Taihi_clicked(true);
  std_msgs::UInt32 z_stage_pos;
  z_stage_pos.data = spinBox_AutoFocus_Zpos->value() - 500;
  stamper_nikon_cmd_z_abs.publish(z_stage_pos);

  ////////////////
  /// Capture Image
  /////////////////
  ros::Duration(3).sleep();
  autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);
  ros::Duration(1).sleep();

  ////////////////
  /// Exchange Position
  /////////////////
  if (!this->on_Exchange_Pos_clicked(true)) {
    ROS_ERROR("Exchange Pos");
    return false;
  }

  ////////////////
  /// Unload Chip
  /////////////////
  if (!this->on_UnloadChip_clicked(true)) {
    ROS_ERROR("UnloadChip");
    return false;
  }
  return true;
}

bool QNode::on_AutoAlign_clicked(bool check) {
  ////////////////////////
  //// Automated Alignment
  ////////////////////////
  if (!this->on_AutoFocus_clicked(true)) {
    if (skip_requested) {
      skip_requested = false;
    } else {
      ROS_ERROR("AutoFocus");
      return false;
    }
  }
  ROS_INFO("%d", fileVer);
  
  if(fileVer == 1) { 
    ////////////////////
    //// Address Flake Fast
    ////////////////////
    if (!this->on_AddressFlakeFast_clicked(true)) {
      ROS_WARN("Error_AddressFlake");
      ///////////////////////////////////
      //// If failed, Switch to Conventional Addressing Algorithm
      ////////////////////////////////////
      skip_requested = false;
      if (!this->on_AddressFlake_clicked(true)) {
        if (skip_requested) {
          skip_requested = false;
        } else {
          ROS_ERROR("AddressFlakeXY5X");
          return false;
        }
      }
    }

    ////////////////////////
    //// Align xytheta
    /////////////////////////
    if (!this->on_AlignFLakeTheta5x_clicked(true)) {
      if (skip_requested) {
        skip_requested = false;
      } else {
        ROS_ERROR("AlignFlakeTheta5x");
        return false;
      }
    }

    //////////////////////
    //// Align XY
    //////////////////////
    if (!this->on_ALignFlakeXY10x_clicked(true)) {
      if (skip_requested) {
        skip_requested = false;
      } else {
        ROS_ERROR("AlignFlakeXY10x");
        return false;
      }
    }

    /////////////////////
    /// Align XY Fine
    ////////////////////
    if (!this->on_AlignFlakeXY10x_Fine_clicked(true)) {
      if (skip_requested) {
        skip_requested = false;
      } else {
        ROS_ERROR("AlignFLakeXY10x_Fine");
        return false;
      }
    }

    /////////
    /// Capture Image
    /////////
    tdmms_autostamp_camera_streamer::CaptureImageTimestamp cap_timestamp;
    cap_timestamp.request.folder = static_cast<const char *>(
        lineEdit_picFolder->text().toStdString().c_str());
    ros::Duration(1).sleep();
    autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);
    ros::Duration(1).sleep();

    //////////
    //// Align by NCC
    //////////
    if (!this->on_AlignFlakeNCC_clicked(true)) {
      if (skip_requested) {
        skip_requested = false;
      } else {
        ROS_ERROR("AlignFLakeNCC");
        return false;
      }
    }
  } else if (fileVer == 2) {
    if (!this->on_AddressFlake2_clicked(true)) {
      if (skip_requested) {
        skip_requested = false;
      } else {
        ROS_ERROR("AddressFlakeFast");
        return false;
      }
    }
    if (!this->on_AlignFlakeTheta5x2_clicked(true)) {
      if (skip_requested) {
        skip_requested = false;
      } else {
        ROS_ERROR("AddressFlakeTheta5x2");
        return false;
      }
    }
    if (!this->on_AlignFlakeXY10x2_clicked(true)) {
      if (skip_requested) {
        skip_requested = false;
      } else {
        ROS_ERROR("AddressFlakeXY10x2");
        return false;
      }
    }
  } else if (fileVer == 3) {
      if (!this->on_AddressFlake3_clicked(true)) {
        if (skip_requested) {
          skip_requested = false;
        } else {
          ROS_ERROR("AddressFlakeFast");
          return false;
        }
      }
      if (!this->on_AlignFlakeTheta5x3_clicked(true)) {
        if (skip_requested) {
          skip_requested = false;
        } else {
          ROS_ERROR("AddressFlakeTheta5x2");
          return false;
        }
      }
    }

  return true;
}

void QNode::on_StartStep_clicked(bool check) {}

void QNode::on_LoadTray_clicked(bool check) {}

bool QNode::on_LoadChip_clicked(bool check) {
  tdmms_autostamp_chip_transfer_action::TransferChipGoal goal;
  goal.isLoading.data = true;
  goal.pocketNo.data = spinBox_PosinChiptray->value();

  if (spinBox_ChiptrayNo->value() == spinBox_ChiptrayNo_1->value()) {
    goal.palletNo.data = spinBox_PalletNo_1->value();
  } else if (spinBox_ChiptrayNo->value() == spinBox_ChiptrayNo_2->value()) {
    goal.palletNo.data = spinBox_PalletNo_2->value();
  } else if (spinBox_ChiptrayNo->value() == spinBox_ChiptrayNo_3->value()) {
    goal.palletNo.data = spinBox_PalletNo_3->value();
  } else {
    ROS_ERROR("Pallte No");
    return false;
  }
  goal.rotangle_deg.data = -doubleSpinBox_Rotangle->value() - 180.0;
  ac_chip_transfer.sendGoal(goal);
  while (true) {
    // qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state = ac_chip_transfer.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_ERROR("Failed to Detect Edge trying next edge... ");
      return false;
    }
  }
  return true;
}

bool QNode::on_CaptureImage(bool check) {
  tdmms_autostamp_camera_streamer::CaptureImageTimestamp cap_timestamp;
  cap_timestamp.request.folder = static_cast<const char *>(
      lineEdit_picFolder->text().toStdString().c_str());
  autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);
  return true;
}

void QNode::on_MoveLinearStage_clicked(bool check) {
  transfer_linear_yamaha::AbsMove moveto;
  moveto.request.targetPose.position.x = doubleSpinBox_LinearStage->value();
  moveto.request.targetPose.position.y = 0;
  moveto.request.targetPose.position.z = 0;
  transfer_linear_cmd_abs_move.call(moveto);
}

void QNode::on_Stamp_Taihi_clicked(bool check) {
  stamper_om_xy::velocity vel;
  vel.velocity_low = 1000;
  vel.velocity_high = 15000;
  vel.accl = 100;
  stamper_om_xy_cmd_vel.publish(vel);
  stamper_sample_xy_cmd_vel.publish(vel);

  ros::Duration(0.5).sleep();

  geometry_msgs::Point pnt;
  pnt.x = 80000;
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);

  pnt.x = -100000;
  pnt.y = 150;
  stamper_sample_xy_cmd_stp.publish(pnt);

  std_srvs::Empty emp_srv;
  stamper_sample_xy_wait_for_stop_client.call(emp_srv);
  stamper_om_xy_wait_for_stop_client.call(emp_srv);
  ros::Duration(1).sleep();

  pnt.x = 10000;
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);

  pnt.x = 10000;
  pnt.y = 200;
  stamper_sample_xy_cmd_stp.publish(pnt);

  stamper_sample_xy_wait_for_stop_client.call(emp_srv);
  stamper_om_xy_wait_for_stop_client.call(emp_srv);
  ros::Duration(1).sleep();
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


  ros::Duration(0.5).sleep();

  geometry_msgs::Point pnt;
  pnt.x = -100000;
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);

  pnt.x = 80000;
  pnt.y = -550;
  stamper_sample_xy_cmd_stp.publish(pnt);

  std_srvs::Empty emp_srv;
  stamper_sample_xy_wait_for_stop_client.call(emp_srv);
  stamper_om_xy_wait_for_stop_client.call(emp_srv);
  ros::Duration(1).sleep();

  pnt.x = 10000;
  pnt.y = 0;
  stamper_om_xy_cmd_stp.publish(pnt);

  pnt.x = 10000;
  pnt.y = 200;
  stamper_sample_xy_cmd_stp.publish(pnt);

  stamper_sample_xy_wait_for_stop_client.call(emp_srv);
  stamper_om_xy_wait_for_stop_client.call(emp_srv);
  ros::Duration(1).sleep();
}

bool QNode::move_Alignment_Pos(int zpos) {
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
    return false;
  }
  if (!(pos.response.pos < 6000)) {
    ROS_ERROR("Z stage not located at home position");
    return false;
  }
  keyence_mask_fwd_publisher.publish(emp);

  ros::Duration(25).sleep();

  keyence_om_fwd_publisher.publish(emp);

  vel_sample_z.velocity_low = 1000;
  vel_sample_z.velocity_high = 15000;
  vel_sample_z.accl = 100;
  stamper_sample_z_cmd_vel.publish(vel_sample_z);

  pnt.x = pos_sample_xy_x;
  pnt.y = pos_sample_xy_y;
  stamper_sample_xy_cmd_abs.publish(pnt);

  pnt.x = zpos;//485000;//585000;//585000;//585000;
  pnt.y = 0;
  stamper_sample_z_cmd_abs.publish(pnt);
  stamper_sample_z_wait_for_stop_client.call(emp_srv);
  stamper_sample_xy_wait_for_stop_client.call(emp_srv);
  ros::Duration(1).sleep();
  keyence_sample_vacuum_on_publisher.publish(emp);

  return true;
}

bool QNode::on_Alignment_Pos_clicked(bool check) {
  move_Alignment_Pos(585000);
  return true;
}

bool QNode::on_Alignment_Pos_Low_clicked(bool check) {
  move_Alignment_Pos(485000);
  return true;
}

bool QNode::on_Exchange_Pos_clicked(bool check) {
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
    return false;
  }
  if (!(pos.response.pos < 10000)) {
    ROS_ERROR("Z stage not located at home position");
    return false;
  }

  keyence_mask_back_publisher.publish(emp);
  ros::Duration(0.1).sleep();
  keyence_sample_vacuum_off_publisher.publish(emp);
  stamper_sample_xy_wait_for_stop_client.call(pos);
  ros::Duration(25).sleep();
  return true;
}

bool QNode::on_UnloadChip_clicked(bool check) {
  tdmms_autostamp_chip_transfer_action::TransferChipGoal goal;
  goal.isLoading.data = false;
  goal.pocketNo.data = spinBox_PosinChiptray->value();
  if (spinBox_ChiptrayNo->value() == spinBox_ChiptrayNo_1->value()) {
    goal.palletNo.data = spinBox_PalletNo_1->value();
  } else if (spinBox_ChiptrayNo->value() == spinBox_ChiptrayNo_2->value()) {
    goal.palletNo.data = spinBox_PalletNo_2->value();
  } else if (spinBox_ChiptrayNo->value() == spinBox_ChiptrayNo_3->value()) {
    goal.palletNo.data = spinBox_PalletNo_3->value();
  } else {
    ROS_ERROR("Pallte No");
    return false;
  }
  goal.rotangle_deg.data = -doubleSpinBox_Rotangle->value() - 180.0;
  ac_chip_transfer.sendGoal(goal);
  while (true) {
    // qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state = ac_chip_transfer.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_ERROR("Failed to Detect Edge trying next edge... ");
      return false;
    }
  }
  return true;
}


int QNode::SQLSelectSearchID2(unsigned long long id_image) {
  QSqlQuery q("", db2);
  q.prepare(
      "select idsearch_fk from 2dmms_db.image where "
      "idimage = ?");
  q.addBindValue(id_image);
  if (!q.exec()) {
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
    return false;
  } else {
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
  }
  q.next();

  return q.value(0).toInt();
}

int QNode::SQLSelectChipID2(unsigned long long id_image) {
  QSqlQuery q("", db2);
  q.prepare(
      "select idchip_fk from 2dmms_db.image where "
      "idimage = ?");
  q.addBindValue(id_image);
  if (!q.exec()) {
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
    return false;
  } else {
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
  }
  q.next();

  return q.value(0).toInt();
}

bool QNode::SQLSelectPositionXY2(int *point_x_target, int *point_y_target,
                                 unsigned long long id_image) {
  QSqlQuery q("", db2);
  q.prepare(
      "select point_x, point_y from 2dmms_db.image where "
      "idimage = ?");
  q.addBindValue(id_image);
  if (!q.exec()) {
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
    return false;
  } else {
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
  }
  q.next();
  *point_x_target = q.value(0).toInt();
  *point_y_target = q.value(1).toInt();
  return true;
}

bool QNode::SQLSelectPositionXY(int *point_x_target, int *point_y_target,
                                QString filename) {
  QSqlQuery q("", db);
  q.prepare(
      "select position_x, position_y from flake_images where "
      "filename_flake_image = ?");
  q.addBindValue(filename);
  if (!q.exec()) {
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
    return false;
  } else {
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
  }
  q.next();
  
  *point_x_target = q.value(0).toInt();
  *point_y_target = q.value(1).toInt();
  return true;
}

QString QNode::SQLSelectFilenameFlakeImage(int position_x, int position_y,
                                           int search_id, int objective_lens) {
  QSqlQuery q("", db);
  q.prepare(
      "select filename_flake_image from flake_images where position_x = "
      "? and position_y = ? and search_id_search = ? and "
      "objective_lens_id_objective_lens_fk = ?");
  q.addBindValue(position_x);
  q.addBindValue(position_y);
  q.addBindValue(search_id);
  q.addBindValue(objective_lens);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
  q.next();

  return q.value(0).toString();
}

QString QNode::SQLSelectFilenameFlakeImage2(int position_x, int position_y,
                                            int search_id, int chip_id, int objective_lens) {
  QSqlQuery q("", db2);
  q.prepare(
      "select point_x, point_y, s3_url"
      " from 2dmms_db.image_align where "
      " idchip_fk = ? and idsearch_fk = ? and idlens_fk = ?"
      " order by (Pow(point_x-(?), 2) + pow(point_y- (?), 2)) asc"
            );
  q.addBindValue(chip_id);
  q.addBindValue(search_id);
  q.addBindValue(objective_lens);
  q.addBindValue(position_x);
  q.addBindValue(position_y);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
  q.next();

  return q.value(2).toString();
}


int QNode::SQLSelectSearchID(QString filename) {
  QSqlQuery q("", db);
  q.prepare(
      "select search_id_search from flake_images "
      "where filename_flake_image = ?");
  q.addBindValue(filename);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
  q.next();
  return q.value(0).toInt();
}

bool QNode::on_AddressFlakeFast_clicked(bool check) {
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  QString filename_target;
  int point_x_target, point_y_target;

  filename_target = lineEdit_Filename->text();
  SQLSelectPositionXY(&point_x_target, &point_y_target, filename_target);

  ////////////////////////////////////////////////////////
  /// Goal Status
  ///////////////////////////////////////////////////////
  tdmms_autostamp_flake_positioner_action_fast::PositionFlakeFastGoal goal;
  goal.filename_target.data = filename_target.toStdString();
  goal.point_target.x = point_x_target;
  goal.point_target.y = point_y_target;
  goal.id_chiptray.data = spinBox_ChiptrayNo->value();
  goal.pos_in_chiptray.data = spinBox_PosinChiptray->value();
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();

  /// idoukaishi
  ac_flake_positioner_fast.sendGoal(goal);

  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state =
        ac_flake_positioner_fast.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_ERROR("Failed to Address Flake Fast, continue processing...");
      return true;
    }
  }
  return false;
}
bool QNode::on_AddressFlake3_clicked(bool check) {
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  QString filename_target;
  filename_target = lineEdit_Filename->text();
  int point_x_target, point_y_target;
  std::string aws_uris_serialized;
  std::string image_oid;
  image_oid = lineEdit_ImageOID->text().toStdString();
  
  tdmms_autostamp_flake_positioner_action_fast3::PositionFlakeFast3Goal goal;
  goal.filename_target.data = filename_target.toStdString();
  goal.point_target.x = lineEdit_Pos_X->text().toInt();
  goal.point_target.y = lineEdit_Pos_Y->text().toInt();

  ROS_INFO("%f,%f", goal.point_target.x, goal.point_target.y);
  std::vector<int> point_xs;
  std::vector<int> point_ys;
  std::vector<std::string> aws_uris;
  
  for(int i = 0 ; i < alignInfos.size(); i++){
    alignInfo elem = alignInfos.at(i);
    if(elem.oid_image == image_oid)
    {
      point_xs.push_back(elem.pos_x);
      point_ys.push_back(elem.pos_y);
      aws_uris.push_back(elem.aws_uri);
      //printf("%d, %d, %s \n", elem.pos_x, elem.pos_y, elem.aws_uri.c_str());
    }
  }

  std::stringstream ss;
  goal.pos_xs.data.resize(point_xs.size());
  goal.pos_ys.data.resize(point_ys.size());

  for(int i = 0 ; i < point_xs.size(); i++){
    goal.pos_xs.data[i] = point_xs.at(i);
    goal.pos_ys.data[i] = point_ys.at(i);
    ss << aws_uris.at(i) << ';';
  }
  goal.aws_uris.data = ss.str();
  ac_flake_positioner_fast3.sendGoal(goal);
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state =
        ac_flake_positioner_fast3.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Align XYtheta, preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Failed to Align XYtheta, continue process..");
      return true;
    }
  }
  return false;
}

bool QNode::on_AddressFlake2_clicked(bool check) {
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  QString filename_target;
  unsigned long long id_image;
  int id_search;
  int id_chip;

  filename_target = lineEdit_Filename->text();
  id_image = spinBox_ImageID->value();
  id_search = SQLSelectSearchID2(id_image);
  id_chip = SQLSelectChipID2(id_image);
  int point_x_target, point_y_target;
  SQLSelectPositionXY2(&point_x_target, &point_y_target, id_image);

  ////////////////////////////////////////////////////////
  /// Goal Status
  ///////////////////////////////////////////////////////
  tdmms_autostamp_flake_positioner_action_fast2::PositionFlakeFast2Goal goal;
  goal.filename_target.data = filename_target.toStdString();
  goal.point_target.x = point_x_target;
  goal.point_target.y = point_y_target;
  goal.id_chiptray.data = spinBox_ChiptrayNo->value();
  goal.pos_in_chiptray.data = spinBox_PosinChiptray->value();
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  goal.id_image.data = id_image;
  goal.id_search.data = id_search;
  goal.id_chip.data = id_chip;
  /// idoukaishi
  ac_flake_positioner_fast2.sendGoal(goal);

  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state =
        ac_flake_positioner_fast2.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_ERROR("Failed to Address Flake Fast, continue processing...");
      return true;
    }
  }
  return false;
}

bool QNode::on_AddressFlake_clicked(bool check) {
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  geometry_msgs::Point point_origin;
  geometry_msgs::Point point_target;
  QStringList filename_origin;
  QString filename_target;
  double point_x_origin[4096];
  double point_y_origin[4096];
  int point_x_target, point_y_target;

  /*****************************************************************************
   ** SQL Query
   *****************************************************************************/
  filename_target = lineEdit_Filename->text();
  SQLSelectPositionXY(&point_x_target, &point_y_target, filename_target);

  //////////////////////////////////////////
  /// Extract Origin Images
  /////////////////////////////////////////
  QSqlQuery q("", db);
  q.prepare(
      "select id_search_chip_fk, pos_origin_x, pos_origin_y, "
      "filename_origin_image from mydb.origin_images inner join "
      "flake_images on origin_images.id_search_fk = "
      "flake_images.search_id_search and origin_images.id_search_chip_fk "
      "= flake_images.search_id_chip_fk where filename_flake_image = ?");
  q.addBindValue(filename_target);
  if (!q.exec())
    printf("QUERY ERROR, ERROR:%s SENT:%s\n",
           q.lastError().text().toStdString().c_str(),
           q.lastQuery().toStdString().c_str());
  else
    printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());

  int i = 0;
  while (q.next()) {
    point_x_origin[i] = static_cast<double>(q.value(1).toInt());
    point_y_origin[i] = static_cast<double>(q.value(2).toInt());
    filename_origin << q.value(3).toString();
    i++;
  }

  /*****************************************************************************
   ** Extract Origin Image with Longest Edge
   *****************************************************************************/
  HObject ho_ImageModel, ho_ImageModelGray;
  HObject ho_ModelRegion;
  HTuple hv_Area, hv_Row, hv_Column;
  HObject ho_ImageModelGrayReduced, ho_ModelContours;
  HTuple hv_ModelId, hv_Length, hv_Length_sum, hv_Length_tuple, hv_i;
  HTuple hv_LengthMax, hv_ind, hv_Exception;
  int edge_count = 0;
  for (i = 0; i < filename_origin.size(); ++i) {
    try {
      ReadImage(&ho_ImageModel,
                HTuple(filename_origin.at(i).toStdString().c_str()));
      Rgb1ToGray(ho_ImageModel, &ho_ImageModelGray);
      GenRectangle1(&ho_ModelRegion, 200, 400, 1086 - 200, 2040 - 400);
      AreaCenter(ho_ModelRegion, &hv_Area, &hv_Row, &hv_Column);
      ReduceDomain(ho_ImageModelGray, ho_ModelRegion,
                   &ho_ImageModelGrayReduced);
      CreateScaledShapeModel(
          ho_ImageModelGrayReduced, 6, HTuple(-180).TupleRad(),
          HTuple(360).TupleRad(), HTuple(0.1452).TupleRad(), 0.81, 0.99, 0.0025,
          (HTuple("point_reduction_medium").Append("no_pregeneration")),
          "use_polarity", ((HTuple(38).Append(52)).Append(86)), 4, &hv_ModelId);
      GetShapeModelContours(&ho_ModelContours, hv_ModelId, 1);
      LengthXld(ho_ModelContours, &hv_Length);
      TupleSum(hv_Length, &hv_Length_sum);
      hv_Length_tuple[i] = hv_Length_sum;
      ClearShapeModel(hv_ModelId);
      edge_count++;
    }
    catch (HalconCpp::HException &HDevExpDefaultException) {
      HDevExpDefaultException.ToHTuple(&hv_Exception);
      hv_Length_tuple[i] = -1;
      continue;
    }
  }
  HTuple hv_sortedIndex, hv_sortedIndex_inv;
  TupleSortIndex(hv_Length_tuple, &hv_sortedIndex);
  TupleInverse(hv_sortedIndex, &hv_sortedIndex_inv);

  for (i = 0; i < edge_count; i++) {
    hv_ind = hv_sortedIndex_inv[i];
    ////////////////////////////////////////////////////////
    /// Goal Status
    ///////////////////////////////////////////////////////
    tdmms_autostamp_flake_positioner_action::PositionFlakeGoal goal;
    goal.filename_origin.data =
        filename_origin[static_cast<int>(hv_ind)].toStdString();
    goal.filename_target.data = filename_target.toStdString();
    goal.point_origin.x = point_x_origin[static_cast<int>(hv_ind)];
    goal.point_origin.y = point_y_origin[static_cast<int>(hv_ind)];
    goal.point_target.x = point_x_target;
    goal.point_target.y = point_y_target;

    //////////////////////////////////////////////////////////////////////
    /// Determine Movement Direction
    ////////////////////////////////////////////////////////////////////
    HObject ho_image, ho_imageGray, ho_region, ho_connectedregion;
    HObject ho_selectedregion;
    HTuple hv_width, hv_height, hv_area, hv_row1, hv_column1;
    HTuple hv_area_max, hv_isInside_TopLeft, hv_isInside_TopRight;
    HTuple hv_isInside_BottomLeft, hv_isInside_BottomRight;
    HTuple hv_orientation;

    ReadImage(&ho_image,
              filename_origin[static_cast<int>(hv_ind)].toStdString().c_str());
    GetImageSize(ho_image, &hv_width, &hv_height);
    Rgb1ToGray(ho_image, &ho_imageGray);
    Threshold(ho_imageGray, &ho_region, 0, 50);
    Connection(ho_region, &ho_connectedregion);
    AreaCenter(ho_connectedregion, &hv_area, &hv_row1, &hv_column1);
    TupleMax(hv_area, &hv_area_max);
    SelectShape(ho_connectedregion, &ho_selectedregion, "area", "and",
                hv_area_max, hv_area_max);

    TestRegionPoint(ho_selectedregion, 0, 0, &hv_isInside_TopLeft);
    TestRegionPoint(ho_selectedregion, 0, hv_width - 1, &hv_isInside_TopRight);
    TestRegionPoint(ho_selectedregion, hv_height - 1, 0,
                    &hv_isInside_BottomLeft);
    TestRegionPoint(ho_selectedregion, hv_height - 1, hv_width - 1,
                    &hv_isInside_BottomRight);
    if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 1)
                                .TupleAnd(hv_isInside_TopRight == 1))
                         .TupleAnd(hv_isInside_BottomLeft == 1))
                  .TupleAnd(hv_isInside_BottomRight == 0))) {
      hv_orientation = "TopLeft";
    } else if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 1)
                                       .TupleAnd(hv_isInside_TopRight == 1))
                                .TupleAnd(hv_isInside_BottomLeft == 0))
                         .TupleAnd(hv_isInside_BottomRight == 1))) {
      hv_orientation = "TopRight";
    } else if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 1)
                                       .TupleAnd(hv_isInside_TopRight == 0))
                                .TupleAnd(hv_isInside_BottomLeft == 1))
                         .TupleAnd(hv_isInside_BottomRight == 1))) {
      hv_orientation = "BottomLeft";
    } else if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 0)
                                       .TupleAnd(hv_isInside_TopRight == 1))
                                .TupleAnd(hv_isInside_BottomLeft == 1))
                         .TupleAnd(hv_isInside_BottomRight == 1))) {
      hv_orientation = "BottomRight";
    } else if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 1)
                                       .TupleAnd(hv_isInside_TopRight == 1))
                                .TupleAnd(hv_isInside_BottomLeft == 0))
                         .TupleAnd(hv_isInside_BottomRight == 0))) {
      hv_orientation = "Top";
    } else if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 0)
                                       .TupleAnd(hv_isInside_TopRight == 0))
                                .TupleAnd(hv_isInside_BottomLeft == 1))
                         .TupleAnd(hv_isInside_BottomRight == 1))) {
      hv_orientation = "Bottom";
    } else if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 1)
                                       .TupleAnd(hv_isInside_TopRight == 0))
                                .TupleAnd(hv_isInside_BottomLeft == 1))
                         .TupleAnd(hv_isInside_BottomRight == 0))) {
      hv_orientation = "Left";
    } else if (0 != (HTuple(HTuple(HTuple(hv_isInside_TopLeft == 0)
                                       .TupleAnd(hv_isInside_TopRight == 1))
                                .TupleAnd(hv_isInside_BottomLeft == 0))
                         .TupleAnd(hv_isInside_BottomRight == 1))) {
      hv_orientation = "Right";
    }

    if (0 != (hv_orientation == HTuple("TopLeft"))) {
      goal.delta_stp.x = 10000;
      goal.delta_stp.y = 0;
    } else if (0 != (hv_orientation == HTuple("TopRight"))) {
      goal.delta_stp.x = 0;
      goal.delta_stp.y = 10000;
    } else if (0 != (hv_orientation == HTuple("BottomLeft"))) {
      goal.delta_stp.x = 0;
      goal.delta_stp.y = -10000;
    } else if (0 != (hv_orientation == HTuple("BottomRight"))) {
      goal.delta_stp.x = -10000;
      goal.delta_stp.y = 0;
    } else if (0 != (hv_orientation == HTuple("Top"))) {
      goal.delta_stp.x = -5000;
      goal.delta_stp.y = 5000;
    } else if (0 != (hv_orientation == HTuple("Bottom"))) {
      goal.delta_stp.x = 5000;
      goal.delta_stp.y = -5000;
    } else if (0 != (hv_orientation == HTuple("Left"))) {
      goal.delta_stp.x = -5000;
      goal.delta_stp.y = -5000;
    } else if (0 != (hv_orientation == HTuple("Right"))) {
      goal.delta_stp.y = 5000;
      goal.delta_stp.x = 5000;
    }

    double delta_x, delta_y;
    double rotangle = doubleSpinBox_Rotangle->value();
    delta_x = static_cast<double>(goal.delta_stp.x);
    delta_y = static_cast<double>(goal.delta_stp.y);

    goal.delta_stp.x = static_cast<int>(delta_x * cos(rotangle / 180.0 * 3.14) -
                                        delta_y * sin(rotangle / 180.0 * 3.14));
    goal.delta_stp.y = static_cast<int>(delta_x * sin(rotangle / 180.0 * 3.14) +
                                        delta_y * cos(rotangle / 180.0 * 3.14));

    goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
    //////////////////////////////////////////////
    ////// Send Goal to Action Server
    //////////////////////////////////////////////
    ac_flake_positioner.sendGoal(goal);

    while (true) {
      qApp->processEvents();
      ros::Duration(0.1).sleep();
      actionlib::SimpleClientGoalState state = ac_flake_positioner.getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return true;
      } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_WARN("Position Flake preempted");
        return false;
      } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_WARN("Failed to Position Flake, Continue process...");
        return true;
      }
    }
  }
  return false;
}

bool QNode::on_AlignFlakeNCC_clicked(bool check) {
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  QString filename_target = lineEdit_Filename->text();

  tdmms_autostamp_flake_aligner_ncc_action::AlignFlakeNCCGoal goal;

  goal.filename_target.data = filename_target.toStdString();
  goal.point_offset.x = lineEdit_Xofs->text().toInt();
  goal.point_offset.y = lineEdit_Yofs->text().toInt();
  goal.theta_rad.data =
      static_cast<double>(doubleSpinBox_Rotangle->value() / 180 * 3.141592);
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  goal.scale_factor_x.data =
      static_cast<double>(doubleSpinBox_Align_NCC_Scale_X->value());
  goal.scale_factor_y.data =
      static_cast<double>(doubleSpinBox_Align_NCC_Scale_Y->value());
  ac_flake_aligner_ncc.sendGoal(goal);

  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state = ac_flake_aligner_ncc.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Flake Alignment NCC Preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Failed to Align Flake NCC, Continue Process..");
      return true;
    }
  }
  return false;
}

bool QNode::on_AlignFlakeXY10x_Fine_clicked(bool check) {
  /*****************************************************************************
   ** Extract Filename
   *****************************************************************************/
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  int point_x_target, point_y_target;
  int search_id;
  QString filename_target;
  QString filename_target_align;

  filename_target = lineEdit_Filename->text();

  SQLSelectPositionXY(&point_x_target, &point_y_target, filename_target);
  search_id = SQLSelectSearchID(filename_target);
  filename_target_align =
      SQLSelectFilenameFlakeImage(point_x_target, point_y_target, search_id, 2);

  /*****************************************************************************
   ** Preparing Action Library
   *****************************************************************************/
  tdmms_autostamp_flake_aligner_xy10x_fine_action::AlignFlakeXY10x_FineGoal
      goal;

  goal.filename_target.data = filename_target_align.toStdString();
  goal.point_offset.x = lineEdit_Xofs->text().toInt();
  goal.point_offset.y = lineEdit_Yofs->text().toInt();
  goal.theta_rad.data =
      static_cast<double>(doubleSpinBox_Rotangle->value() / 180 * 3.141592);
  goal.alignment_offset_theta.x = spinBox_Align_Offset_Theta_X->value();
  goal.alignment_offset_theta.y = spinBox_Align_Offset_Theta_Y->value();
  goal.alignment_offset.x = spinBox_Align_Offset_X->value();
  goal.alignment_offset.y = spinBox_Align_Offset_Y->value();
  goal.scaling_factor.data = doubleSpinBox_Align_Offset_Scale->value();
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  ac_flake_aligner_xy10x_fine.sendGoal(goal);
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state =
        ac_flake_aligner_xy10x_fine.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Flake Alignment at 10x_fine preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Flake Alignment at 10x_fine aborted, continue processing...");
      return true;
    }
  }
  return false;
}

bool QNode::on_ALignFlakeXY10x_clicked(bool check) {
  /*****************************************************************************
   ** Extract Filename
   *****************************************************************************/
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  int point_x_target, point_y_target;
  int search_id;
  QString filename_target, filename_target_align;

  filename_target = lineEdit_Filename->text();
  SQLSelectPositionXY(&point_x_target, &point_y_target, filename_target);
  search_id = SQLSelectSearchID(filename_target);
  filename_target_align =
      SQLSelectFilenameFlakeImage(point_x_target, point_y_target, search_id, 1);

  /*****************************************************************************
   ** Preparing Action Library
   *****************************************************************************/
  tdmms_autostamp_flake_aligner_xy10x_action::AlignFlakeXY10xGoal goal;

  goal.filename_target.data = filename_target_align.toStdString();
  goal.point_offset.x = lineEdit_Xofs->text().toInt();
  goal.point_offset.y = lineEdit_Yofs->text().toInt();
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  ac_flake_aligner_xy10x.sendGoal(goal);
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state = ac_flake_aligner_xy10x.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Flake Alignment at 10x preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Flake Alignment at 10x aborted, continue processing...");
      return true;
    }
  }
  return false;
}

bool QNode::on_AlignFlakeXY10x2_clicked(bool check) {
  ROS_INFO("A");
  /*****************************************************************************
   ** Extract Filename
   *****************************************************************************/
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  int point_x_target, point_y_target;
  QString filename_target, filename_target_align;
  unsigned long long id_image;
  int id_search;
  int id_chip;
  ROS_INFO("AlignFlake");
  filename_target = lineEdit_Filename->text();
  id_image = spinBox_ImageID->value();
  id_search = SQLSelectSearchID2(id_image);
  id_chip = SQLSelectChipID2(id_image);
  SQLSelectPositionXY2(&point_x_target, &point_y_target, id_image);
  filename_target_align =
      SQLSelectFilenameFlakeImage2(point_x_target, point_y_target, id_search, id_chip, 1);

  /*filename_target = lineEdit_Filename->text();
  SQLSelectPositionXY(&point_x_target, &point_y_target, filename_target);
  search_id = SQLSelectSearchID(filename_target);
  filename_target_align =
      SQLSelectFilenameFlakeImage(point_x_target, point_y_target, search_id, 1);
  */
  /*****************************************************************************
   ** Preparing Action Library
   *****************************************************************************/
  tdmms_autostamp_flake_aligner_xy10x2_action::AlignFlakeXY10x2Goal goal;

  goal.filename_target.data = filename_target_align.toStdString();
  goal.point_offset.x = lineEdit_Xofs->text().toInt();
  goal.point_offset.y = lineEdit_Yofs->text().toInt();
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  ac_flake_aligner_xy10x2.sendGoal(goal);
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state = ac_flake_aligner_xy10x2.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Flake Alignment at 10x preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Flake Alignment at 10x aborted, continue processing...");
      return true;
    }
  }
  return false;
}

bool QNode::on_AlignFLakeTheta5x_clicked(bool check) {
  /*****************************************************************************
   ** Extract Filename
   *****************************************************************************/
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  int point_x_target, point_y_target;
  int search_id;
  QString filename_target, filename_target_align;

  filename_target = lineEdit_Filename->text();
  SQLSelectPositionXY(&point_x_target, &point_y_target, filename_target);
  search_id = SQLSelectSearchID(filename_target);
  filename_target_align =
      SQLSelectFilenameFlakeImage(point_x_target, point_y_target, search_id, 0);

  /*****************************************************************************
   ** Preparing Action Library
   *****************************************************************************/
  tdmms_autostamp_flake_aligner_xytheta5x_action::AlignFlakeXYTheta5xGoal goal;
  goal.filename_target.data = filename_target_align.toStdString();
  goal.point_offset.x = lineEdit_Xofs->text().toInt();
  goal.point_offset.y = lineEdit_Yofs->text().toInt();
  goal.theta_rad.data =
      static_cast<double>(doubleSpinBox_Rotangle->value() / 180 * 3.141592);
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  ac_flake_aligner_xytheta5x.sendGoal(goal);
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state =
        ac_flake_aligner_xytheta5x.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Align XYtheta, preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Failed to Align XYtheta, continue process..");
      return true;
    }
  }
  return false;
}

bool QNode::on_AlignFlakeTheta5x3_clicked(bool check) {
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }

  QString filename_target;
  filename_target = lineEdit_Filename->text();
  int point_x_target, point_y_target;
  std::string aws_uris_serialized;
  std::string image_oid;
  image_oid = lineEdit_ImageOID->text().toStdString();
  
  std::vector<int> point_xs;
  std::vector<int> point_ys;
  std::vector<std::string> aws_uris;
  
  for(int i = 0 ; i < alignInfos.size(); i++){
    alignInfo elem = alignInfos.at(i);
    if(elem.oid_image == image_oid)
    {
      point_xs.push_back(elem.pos_x);
      point_ys.push_back(elem.pos_y);
      aws_uris.push_back(elem.aws_uri);
      printf("%d, %d, %s \n", elem.pos_x, elem.pos_y, elem.aws_uri.c_str());
    }
  }

  int target_x = lineEdit_Pos_X->text().toInt();
  int target_y = lineEdit_Pos_Y->text().toInt();

  int min_distance = std::pow(target_x-point_xs.at(0), 2) + std::pow(target_y-point_ys.at(0), 2);
  int index = 0;
  for(int i = 0 ; i < point_xs.size(); i ++) {
    int distance = std::pow(target_x-point_xs.at(i), 2) + std::pow(target_y-point_ys.at(i), 2);
    if(min_distance > distance){
      index = i;
      min_distance = distance;
    }
  }

  tdmms_autostamp_flake_aligner_xytheta5x3_action::AlignFlakeXYTheta5x3Goal goal;
  std::cout << aws_uris.at(index) << std::endl;
  goal.filename_target.data = aws_uris.at(index);
  goal.point_offset.x = lineEdit_Xofs->text().toInt();
  goal.point_offset.y = lineEdit_Yofs->text().toInt();
  goal.theta_rad.data =
      static_cast<double>(doubleSpinBox_Rotangle->value() / 180 * 3.141592);
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  ac_flake_aligner_xytheta5x3.sendGoal(goal);
   while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state =
        ac_flake_aligner_xytheta5x3.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Align XYtheta, preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Failed to Align XYtheta, continue process..");
      return true;
    }
  }
  return false;
}

bool QNode::on_AlignFlakeTheta5x2_clicked(bool check) {
  /*****************************************************************************
   ** Extract Filename
   *****************************************************************************/
  if (lineEdit_Filename->text() == tr("")) {
    return false;
  }
  /*int point_x_target, point_y_target;
  int search_id;
  QString filename_target, filename_target_align;*/

  QString filename_target, filename_target_align;
  unsigned long long id_image;
  int id_search;
  int id_chip;
  ROS_INFO("AlignFlake");
  filename_target = lineEdit_Filename->text();
  id_image = spinBox_ImageID->value();
  id_search = SQLSelectSearchID2(id_image);
  id_chip = SQLSelectChipID2(id_image);
  int point_x_target, point_y_target;
  SQLSelectPositionXY2(&point_x_target, &point_y_target, id_image);
  filename_target_align =
      SQLSelectFilenameFlakeImage2(point_x_target, point_y_target, id_search, id_chip, 0);

  /*
  filename_target = lineEdit_Filename->text();
  SQLSelectPositionXY(&point_x_target, &point_y_target, filename_target);
  search_id = SQLSelectSearchID(filename_target);
  filename_target_align =
      SQLSelectFilenameFlakeImage(point_x_target, point_y_target, search_id, 0);
  */
  ROS_INFO("%s", filename_target_align.toStdString().c_str());
  /*****************************************************************************
   ** Preparing Action Library
   *****************************************************************************/
  tdmms_autostamp_flake_aligner_xytheta5x2_action::AlignFlakeXYTheta5x2Goal goal;
  goal.filename_target.data = filename_target_align.toStdString();
  goal.point_offset.x = lineEdit_Xofs->text().toInt();
  goal.point_offset.y = lineEdit_Yofs->text().toInt();
  goal.theta_rad.data =
      static_cast<double>(doubleSpinBox_Rotangle->value() / 180 * 3.141592);
  goal.focuspos.data = spinBox_AutoFocus_Zpos->value();
  ac_flake_aligner_xytheta5x2.sendGoal(goal);
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state =
        ac_flake_aligner_xytheta5x2.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Align XYtheta, preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Failed to Align XYtheta, continue process..");
      return true;
    }
  }
  return false;
}

void QNode::on_AlignFlakeTheta10x_clicked(bool check) {}

void QNode::on_SetZstage_clicked(bool check) {
  stamper_sample_z::GetCurrentPos pos;
  stamper_sample_z_get_current_pos_client.call(pos);
  spinBox_Auto_Stamp_ZatStamp->setValue(pos.response.pos);
}

bool QNode::on_Stamp_clicked(bool check) {
  tdmms_autostamp_stamp_action::StampGoal goal;
  goal.up_speed.data = static_cast<int>(spinBox_Auto_Stamp_Up_Speed->value());
  goal.down_speed.data =
      static_cast<int>(spinBox_Auto_Stamp_Down_Speed->value());
  goal.stamp_duration.data = static_cast<int>(spinBox_Auto_Stamp_Time->value());
  goal.stamp_load.data = static_cast<int>(spinBox_Auto_Stamp_Load->value());
  goal.om_position_at_stamp.data =
      static_cast<int>(spinBox_Auto_Stamp_OMatStamp->value());
  goal.z_stage_position_at_stamp.x =
      static_cast<int>(spinBox_Auto_Stamp_ZatStamp->value());
  goal.z_stage_position_at_stamp.y =
      static_cast<int>(spinBox_Auto_Stamp_ZatStamp->value());
  goal.folder.data = lineEdit_picFolder->text().toStdString().c_str();

  ac_stamp.sendGoal(goal);
  while (true) {
    qApp->processEvents();
    ros::Duration(0.1).sleep();
    actionlib::SimpleClientGoalState state = ac_stamp.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_WARN("Stamp Action preempted");
      return false;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_ERROR("Failed to stamp, continue process ");
      return true;
    }
  }
  return false;
}

void QNode::on_UnloadTray_clicked(bool check) {}

void QNode::on_LoadFile_clicked(bool check) {}

void QNode::on_SaveFile_clicked(bool check) {}

void QNode::on_Suspend_clicked(bool check) {}

void QNode::on_Resume_clicked(bool check) {}

}  // namespace tdmms_autostamp_master
