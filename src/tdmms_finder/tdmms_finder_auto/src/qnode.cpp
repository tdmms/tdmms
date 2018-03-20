// Copyright 2016 by S. Masubuchi
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
#include <QMessageBox>
#include <QtGui>
#include <QStandardItemModel>
#include <QFileDialog>
#include <QtSql/QtSql>
#include <QDateEdit>
#include <QDir>

#include <dlfcn.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <adm2/velocity.h>

#include <list>
#include <string>
#include <sstream>
#include <iostream>

#include "../../tdmms_finder_libdecision/include/tdmms_finder_hinterface.h"
#include "HalconCpp.h"
#include "HDevThread.h"
#include "../include/tdmms_finder_auto/qnode.hpp"
#include "../include/tdmms_finder_auto/main_window.hpp"

#ifndef __APPLE__
#include "halconcpp/HalconCpp.h"
#include "halconcpp/HDevThread.h"
#if defined(__linux__) && !defined(NO_EXPORT_APP_MAIN)
#include <X11/Xlib.h>
#endif
#else
#ifndef HC_LARGE_IMAGES
#include <HALCONCpp/HalconCpp.h>
#include <HALCONCpp/HDevThread.h>
#else
#include <HALCONCppxl/HalconCpp.h>
#include <HALCONCppxl/HDevThread.h>
#endif
#include <stdio.h>
#include <HALCON/HpThread.h>
#include <CoreFoundation/CFRunLoop.h>
#endif

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/
namespace tdmms_finder_auto {
using namespace Qt;
/*****************************************************************************
 ** QNode constractor
 *****************************************************************************/
QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv) {
  currentState = Live;
  dlhandle = 0;
  OpenSQLConnection();

  // Parameters
  yokoidou = 1120;
  tateidou = -560;
  syasinnnokazu_yoko = 39;
  syasinnnokazu_tate = 78;
  startpoint.x = -25000;
  startpoint.y = 25000;
  strDefaultImageFolder = QDir::homePath() + "/tdmms_data/images";
}

/*****************************************************************************
 ** ROS Node Initialization
 *****************************************************************************/
bool QNode::init() {
  ros::init(init_argc, init_argv, "tdmms_finder_auto");
  if (!ros::master::check()) {
    return false;
  }

  ///////////////////////////////////////////////////
  /// List detection libraries
  //////////////////////////////////////////////////
  QDir dirProj(QDir::homePath() + "/tdmms_ws/devel/lib");
  QStringList strlFilter;
  strlFilter << "libdec*.so";
  listLibraryFile = dirProj.entryInfoList(strlFilter, QDir::Files);
  for (int i = 0; i < listLibraryFile.size(); i++)
    FinderLibrarycomboBox->addItem(listLibraryFile[i].fileName());

  QStringList wafer_names = SQLGetWaferNames();
  for (int i = 0; i < wafer_names.size(); i++)
    comboBox_Wafer->addItem(wafer_names[i]);

  QStringList crystal_names = SQLGetCrystalNames();
  for (int i = 0; i < crystal_names.size(); i++)
    comboBox_Crystal->addItem(crystal_names[i]);

  QStringList exfoliator_names = SQLGetExfoliatorName();
  for (int i = 0; i < exfoliator_names.size(); i++)
    comboBox_Exfoliator->addItem(exfoliator_names[i]);
  
  on_Button_Add_clicked();
  ros::start();

  ///////////////////////////////////////////////////
  //// initialize ros communications
  ///////////////////////////////////////////////////
  ros::NodeHandle n;
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  adm2_abs_publisher =
      n.advertise<geometry_msgs::Point>("/adm2_master/cmd_abs_move", 1);
  adm2_stp_publisher =
      n.advertise<geometry_msgs::Point>("/adm2_master/cmd_stp_move", 1);
  afc_5_publisher = n.advertise<std_msgs::UInt8>("/afc_5_master/cmd_chx", 1);
  afc_5_publisher_sc0 =
      n.advertise<std_msgs::Empty>("/afc_5_master/cmd_sc0", 1);
  lv_ncnt_n_publisher =
      n.advertise<std_msgs::UInt8>("/lv_ncnt_n_master/cmd_set", 1);
  nic_100a_inten_publisher =
      n.advertise<std_msgs::UInt8>("/nic_100a/cmd_inten", 1);
  cmd_currpos_subscriber =
      n.subscribe("/adm2_master/status", 1, &QNode::currpos_Callback, this);
  joy_subscriber = n.subscribe("joy", 1, &QNode::joy_Callback, this);
  adm2_wait_for_stop_client =
      n.serviceClient<std_srvs::Empty>("/adm2_master/wait_for_stop");
  adm2_vel_publisher = n.advertise<adm2::velocity>("/adm2_master/cmd_vel", 1);
  start();
  return true;
}

bool QNode::init(const std::string& master_url, const std::string& host_url) {
  return true;
}

/*****************************************************************************
 ** QNode Destructor
 *****************************************************************************/
QNode::~QNode() {
  fp_tdmms_finder_CloseWindow();
  dlclose(dlhandle);
  db.close();

  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

/*****************************************************************************
 ** ROS Callback
 *****************************************************************************/
void QNode::currpos_Callback(const geometry_msgs::Point& pnt) {
  currpoint = pnt;
}

/*****************************************************************************
 ** ROS Callback Capture Image
 *****************************************************************************/
void QNode::joy_Callback(const sensor_msgs::Joy& joy_msg) {
  char filename[4028];
  int piccount;
  if (joy_msg.buttons[3] == 1) {
    piccount = spinBox_pic_count->value();
    sprintf(filename, "%s/m_%010d_%010d_%010d_%010d.jpg",
            (char*)lineEdit_picFolder->text().toStdString().c_str(), piccount,
            static_cast<int>(currpoint.x), static_cast<int>(currpoint.y),
            0000000);
    fp_tdmms_finder_SaveImage(filename);
    piccount++;
    spinBox_pic_count->setValue(piccount);
  }
}

/*****************************************************************************
 ** Connect to SQL Server
 *****************************************************************************/
void QNode::OpenSQLConnection(void) {
  QSqlError err;
  db =
      QSqlDatabase::addDatabase(QString("QMYSQL"), QString("Browser%1").arg(1));
  db.setDatabaseName(QString("mydb"));
  db.setHostName(QString("localhost"));
  db.setPort(3306);
  if (!db.open(QString("root"), QString("mlab2dmms"))) {
    err = db.lastError();
    db = QSqlDatabase();
    QSqlDatabase::removeDatabase(QString("Browser%1").arg(1));
  } else {
    ROS_INFO("connected to SQL Server");
  }
}

/*****************************************************************************
 ** Loading Image Recognition Library
 *****************************************************************************/
void QNode::on_FinderLibrarycomboBox_currentIndexChanged(int i) {
  if (dlhandle == 0) {
    dlhandle =
        dlopen(listLibraryFile[0].absoluteFilePath().toAscii(), RTLD_LAZY);
    if (dlhandle == 0) {
      ROS_ERROR("dlerror: %s", dlerror());
      exit(-1);
    } else {
      fp_tdmms_finder_hello =
          (tdmms_finder_hello_ptr)dlsym(dlhandle, "tdmms_finder_hello");
      fp_tdmms_finder_Initialize = (tdmms_finder_Initialize_ptr)dlsym(
          dlhandle, "tdmms_finder_Initialize");
      fp_tdmms_finder_Live =
          (tdmms_finder_Live_ptr)dlsym(dlhandle, "tdmms_finder_Live");
      fp_tdmms_finder_CloseWindow = (tdmms_finder_CloseWindow_ptr)dlsym(
          dlhandle, "tdmms_finder_CloseWindow");
      fp_tdmms_finder_OpenWindow = (tdmms_finder_OpenWindow_ptr)dlsym(
          dlhandle, "tdmms_finder_OpenWindow");
      fp_tdmms_finder_Find =
          (tdmms_finder_Find_ptr)dlsym(dlhandle, "tdmms_finder_Find");
      fp_tdmms_finder_SetParameter = (tdmms_finder_SetParameter_ptr)dlsym(
          dlhandle, "tdmms_finder_SetParameter");
      fp_tdmms_finder_GetParameter = (tdmms_finder_GetParameter_ptr)dlsym(
          dlhandle, "tdmms_finder_GetParameter");
      fp_tdmms_finder_SaveImage =
          (tdmms_finder_SaveImage_ptr)dlsym(dlhandle, "tdmms_finder_SaveImage");
      fp_tdmms_finder_SaveImage_raw =
          (tdmms_finder_SaveImage_raw_ptr)dlsym(dlhandle, "tdmms_finder_SaveImage_raw");
      fp_tdmms_finder_SaveImage_affine =
          (tdmms_finder_SaveImage_affine_ptr)dlsym(
              dlhandle, "tdmms_finder_SaveImage_affine");
      fp_tdmms_finder_ExtractFeatures = (tdmms_finder_ExtractFeatures_ptr)dlsym(
          dlhandle, "tdmms_finder_ExtractFeatures");
      fp_tdmms_finder_SaveRegion = (tdmms_finder_SaveRegion_ptr)dlsym(
          dlhandle, "tdmms_finder_SaveRegion");
      Hcpar param;
      char c_para[] = "comment";
      fp_tdmms_finder_GetParameter(c_para, &param);
      label_comment->setText(param.par.s);
      fp_tdmms_finder_Initialize();
      fp_tdmms_finder_OpenWindow();
      currentState = Live;
    }
  } else {
    char filename_ch[1024];
    QByteArray filename =
        listLibraryFile[static_cast<int>(FinderLibrarycomboBox->currentIndex())]
            .absoluteFilePath()
            .toAscii();
    strcpy(filename_ch, filename.constData());
    filename_ch[strlen(filename.constData())] = '\0';
    currentState = unLive;
    ros::Duration(2.0).sleep();
    fp_tdmms_finder_CloseWindow();
    ros::Duration(2.0).sleep();
    dlclose(dlhandle);
    dlhandle = dlopen(filename_ch, RTLD_LAZY);
    if (dlhandle == 0) {
      ROS_ERROR("dlerror: %s", dlerror());
      exit(-1);
    } else {
      fp_tdmms_finder_hello =
          (tdmms_finder_hello_ptr)dlsym(dlhandle, "tdmms_finder_hello");
      fp_tdmms_finder_Initialize = (tdmms_finder_Initialize_ptr)dlsym(
          dlhandle, "tdmms_finder_Initialize");
      fp_tdmms_finder_Live =
          (tdmms_finder_Live_ptr)dlsym(dlhandle, "tdmms_finder_Live");
      fp_tdmms_finder_CloseWindow = (tdmms_finder_CloseWindow_ptr)dlsym(
          dlhandle, "tdmms_finder_CloseWindow");
      fp_tdmms_finder_OpenWindow = (tdmms_finder_OpenWindow_ptr)dlsym(
          dlhandle, "tdmms_finder_OpenWindow");
      fp_tdmms_finder_Find =
          (tdmms_finder_Find_ptr)dlsym(dlhandle, "tdmms_finder_Find");
      fp_tdmms_finder_SetParameter = (tdmms_finder_SetParameter_ptr)dlsym(
          dlhandle, "tdmms_finder_SetParameter");
      fp_tdmms_finder_GetParameter = (tdmms_finder_GetParameter_ptr)dlsym(
          dlhandle, "tdmms_finder_GetParameter");
      fp_tdmms_finder_SaveImage =
          (tdmms_finder_SaveImage_ptr)dlsym(dlhandle, "tdmms_finder_SaveImage");
      fp_tdmms_finder_SaveImage_affine =
          (tdmms_finder_SaveImage_affine_ptr)dlsym(
              dlhandle, "tdmms_finder_SaveImage_affine");
      fp_tdmms_finder_SaveImage_raw =
          (tdmms_finder_SaveImage_raw_ptr)dlsym(dlhandle, "tdmms_finder_SaveImage_raw");
      fp_tdmms_finder_ExtractFeatures = (tdmms_finder_ExtractFeatures_ptr)dlsym(
          dlhandle, "tdmms_finder_ExtractFeatures");
      fp_tdmms_finder_SaveRegion = (tdmms_finder_SaveRegion_ptr)dlsym(
          dlhandle, "tdmms_finder_SaveRegion");
      Hcpar param;
      char c_para[] = "comment";
      fp_tdmms_finder_GetParameter(c_para, &param);
      label_comment->setText(param.par.s);
      fp_tdmms_finder_Initialize();
      fp_tdmms_finder_OpenWindow();
      currentState = Live;
    }
  }
}

/*****************************************************************************
 ** ROS Main
 *****************************************************************************/
void QNode::run() {
  ros::Rate loop_rate(10);  // 50fps
  while (ros::ok()) {
    if (currentState == Live) {
      fp_tdmms_finder_Live();
      ros::spinOnce();
    } else if (currentState == Check) {
      bool result;
      int pos_x, pos_y;
      unsigned int score;
      fp_tdmms_finder_Find(&result, &pos_x, &pos_y, &score);
    }
  }
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to
                         // roslaunch)
}

void QNode::log(const LogLevel& level, const std::string& msg) {
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

void QNode::on_CheckBox_testmode_stateChanged(int state) {
  if (currentState != Search) {
    if (state == 0) {
      currentState = Live;
    } else if (state == 2) {
      currentState = Check;
    }
  }
}

void QNode::SQLInsertChip(int nofchips, int id_chiptray, int id_wafer,
                          QDateTime datetime, QString exfoliated_place,
                          int id_crystal, int id_exfoliator) {
  int id_chip = SQLGetMaximumChipID();
  int i;
  for (i = 0; i < nofchips; i++) {
    QSqlQuery q("", db);
    id_chip++;
    q.prepare(
        "INSERT INTO mydb.chip (id_chip, id_chiptray_fk, id_wafer_fk, "
        "position_in_chiptray)"
        "VALUES (?, ?, ?, ?)");
    q.bindValue(0, id_chip);
    q.bindValue(1, id_chiptray);
    q.bindValue(2, id_wafer);
    q.bindValue(3, i + 1);
    if (!q.exec())
      ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                q.lastError().text().toStdString().c_str(),
                q.executedQuery().toStdString().c_str());
    else
      ROS_DEBUG("QUERY SUCCESS, SENT:%s",
                q.executedQuery().toStdString().c_str());

    q.prepare(
        "INSERT INTO mydb.exfoliation_parameter (id_chip,"
        "exfoliated_datetime, exfoliated_place, id_crystal_fk, id_exfoliator_fk) "
        "VALUES (?, ?, ?, ?, ?)");
    q.bindValue(0, id_chip);
    q.bindValue(1, datetime.toString("yyyy-MM-dd hh:mm:ss"));
    q.bindValue(2, exfoliated_place);
    q.bindValue(3, id_crystal);
    q.bindValue(4, id_exfoliator);
    if (!q.exec())
      ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                q.lastError().text().toStdString().c_str(),
                q.executedQuery().toStdString().c_str());
    else
      ROS_DEBUG("QUERY SUCCESS, SENT:%s",
                q.executedQuery().toStdString().c_str());
  }
}

void QNode::SQLInsertSearch(int id_search, int id_chip,
                            const char* search_program,
                            int target_layer_num_min,
                            int target_layer_num_max) {
  QSqlQuery q("", db);
  q.prepare(
      "INSERT INTO mydb.search (id_search, id_chip_fk, "
      "datetime_search_started, search_program, target_layer_num_min, "
      "target_layer_num_max ) VALUES (?, ?, CURRENT_TIMESTAMP, ?, ?, ?)");
  q.bindValue(0, id_search);
  q.bindValue(1, id_chip);
  q.bindValue(2, search_program);
  q.bindValue(3, target_layer_num_min);
  q.bindValue(4, target_layer_num_max);
  q.exec();
}

void QNode::SQLInsertOriginImage(int id_origin_image, int id_search,
                                 int id_search_chip, int pos_x, int pos_y,
                                 char* filename) {
  QSqlQuery q("", db);
  q.prepare(
      "INSERT INTO mydb.origin_images (id_origin_image, "
      "id_objective_lens_fk, id_search_fk, id_search_chip_fk, pos_origin_x, "
      "pos_origin_y, filename_origin_image) VALUES (?, ?, ?, ?, ?, ?, ?)");
  q.addBindValue(id_origin_image);
  q.addBindValue(0);
  q.addBindValue(id_search);
  q.addBindValue(id_search_chip);
  q.addBindValue(pos_x);
  q.addBindValue(pos_y);
  q.addBindValue(filename);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
}

void QNode::SQLInsertFlakeImage(int flakecount, int id_objective_lens,
                                char* filename, int point_x, int point_y,
                                int id_search, int id_chip,
                                ObjectFeatures feature) {
  QSqlQuery q("", db);
  q.prepare(
      "INSERT INTO mydb.flake_images (id_flake_images, "
      "objective_lens_id_objective_lens_fk, "
      "table_optical_filter_id_optical_filter_fk, "
      "filename_flake_image, area, position_x, position_y, "
      "search_id_search, search_id_chip_fk, used, deleted, "
      "area_holes, bulkiness, circularity, compactness, "
      "connect_num, contlength, convexity, dist_deviation, "
      "dist_mean, euler_number, height, holes_num, inner_height, "
      "inner_radius, inner_width, max_diameter, moments_i1, "
      "moments_i2, moments_i3, moments_i4, moments_ia, moments_ib, "
      "moments_m02, moments_m02_invar, moments_m03, "
      "moments_m03_invar, moments_m11, moments_m11_invar, "
      "moments_m12, moments_m12_invar, moments_m20, "
      "moments_m20_invar, moments_m21, moments_m21_invar, "
      "moments_m30, moments_m30_invar, moments_phi1, moments_phi2, "
      "moments_psi1, moments_psi2, moments_psi3, moments_psi4, "
      "num_sides, orientation, outer_radius, phi, ra, rb, "
      "rect2_len1, rect2_len2, rect2_phi, rectangularity, "
      "roundness, struct_factor, width, row, row1, row2, "
      "anisometry, n_column, n_column1, n_column2) VALUES (?, ?, "
      "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, "
      "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, "
      "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, "
      "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
  q.addBindValue(flakecount);
  q.addBindValue(id_objective_lens);
  q.addBindValue(0);
  q.addBindValue(filename);
  q.addBindValue(static_cast<double>(feature.area));
  q.addBindValue(static_cast<int>(point_x));
  q.addBindValue(static_cast<int>(point_y));
  q.addBindValue(id_search);
  q.addBindValue(id_chip);
  q.addBindValue(0);
  q.addBindValue(0);
  q.addBindValue(feature.area_holes);
  q.addBindValue(feature.bulkiness);
  q.addBindValue(feature.circularity);
  q.addBindValue(feature.compactness);
  q.addBindValue(feature.connect_num);
  q.addBindValue(feature.contlength);
  q.addBindValue(feature.convexity);
  q.addBindValue(feature.dist_deviation);
  q.addBindValue(feature.dist_mean);
  q.addBindValue(feature.euler_number);
  q.addBindValue(feature.height);
  q.addBindValue(feature.holes_num);
  q.addBindValue(feature.inner_height);
  q.addBindValue(feature.inner_radius);
  q.addBindValue(feature.inner_width);
  q.addBindValue(feature.max_diameter);
  q.addBindValue(feature.moments_i1);
  q.addBindValue(feature.moments_i2);
  q.addBindValue(feature.moments_i3);
  q.addBindValue(feature.moments_i4);
  q.addBindValue(feature.moments_ia);
  q.addBindValue(feature.moments_ib);
  q.addBindValue(feature.moments_m02);
  q.addBindValue(feature.moments_m02_invar);
  q.addBindValue(feature.moments_m03);
  q.addBindValue(feature.moments_m03_invar);
  q.addBindValue(feature.moments_m11);
  q.addBindValue(feature.moments_m11_invar);
  q.addBindValue(feature.moments_m12);
  q.addBindValue(feature.moments_m12_invar);
  q.addBindValue(feature.moments_m20);
  q.addBindValue(feature.moments_m20_invar);
  q.addBindValue(feature.moments_m21);
  q.addBindValue(feature.moments_m21_invar);
  q.addBindValue(feature.moments_m30);
  q.addBindValue(feature.moments_m30_invar);
  q.addBindValue(feature.moments_phi1);
  q.addBindValue(feature.moments_phi2);
  q.addBindValue(feature.moments_psi1);
  q.addBindValue(feature.moments_psi2);
  q.addBindValue(feature.moments_psi3);
  q.addBindValue(feature.moments_psi4);
  q.addBindValue(feature.num_sides);
  q.addBindValue(feature.orientation);
  q.addBindValue(feature.outer_radius);
  q.addBindValue(feature.phi);
  q.addBindValue(feature.ra);
  q.addBindValue(feature.rb);
  q.addBindValue(feature.rect2_len1);
  q.addBindValue(feature.rect2_len2);
  q.addBindValue(feature.rect2_phi);
  q.addBindValue(feature.rectangularity);
  q.addBindValue(feature.roundness);
  q.addBindValue(feature.struct_factor);
  q.addBindValue(feature.width);
  q.addBindValue(feature.row);
  q.addBindValue(feature.row1);
  q.addBindValue(feature.row2);
  q.addBindValue(feature.anisometry);
  q.addBindValue(feature.column);
  q.addBindValue(feature.column1);
  q.addBindValue(feature.column2);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
  return;
}

bool QNode::SQLCheckDuplicate(int id_search, int id_chip,
                              int point_x, int point_y,
                              int thresh) {
  /////////////////////////////////////////////////////
  //// Check if the record at current position is present in database
  ////////////////////////////////////////////////////

  QSqlQuery q("", db);
  q.prepare(
      "SELECT count(*) FROM mydb.flake_images where search_id_search "
      "= ? and search_id_chip_fk = ? and "
      "sqrt(Power(position_x - ?, 2) + Power(position_y - ?, "
      "2)) < ?");
  q.addBindValue(id_search);
  q.addBindValue(id_chip);
  q.addBindValue(point_x);
  q.addBindValue(point_y);
  q.addBindValue(thresh);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
  q.next();
  int duplicate;
  duplicate = q.value(0).toInt();
  if (duplicate == 0) return false;

  ROS_INFO("duplicate:%d", duplicate);
  return true;
}

void QNode::SQLInsertAlignmentImage(char* filename, int pos_alignment_x,
                                    int pos_alignment_y, int id_search,
                                    int id_chip) {
  int id_alignment_image;
  QSqlQuery q("", db);

  q.prepare("SELECT MAX(id_alignment_image) FROM alignment_images");
  q.exec();
  while (q.next()) {
    id_alignment_image = q.value(0).toInt();
    printf("id_alignment_image:%d\n", id_alignment_image);
  }
  id_alignment_image++;

  q.prepare(
      "INSERT INTO mydb.alignment_images (id_alignment_image, "
      "filename_alignment_image, pos_alignment_x, "
      "pos_alignment_y, id_search_fk, "
      "id_search_chip_fk, id_objective_lens_fk "
      ") VALUES (?, ?, ?, ?, ?, "
      "?, ?)");
  q.bindValue(0, id_alignment_image);
  q.bindValue(1, filename);
  q.bindValue(2, pos_alignment_x);
  q.bindValue(3, pos_alignment_y);
  q.bindValue(4, id_search);
  q.bindValue(5, id_chip);
  q.bindValue(6, 0);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
}

QStringList QNode::SQLGetExfoliatorName() {
  QStringList ExfoliatorNames;
  QSqlQuery q("", db);
  q.prepare(
      "SELECT name_first, name_family from exfoliator order by id_exfoliator "
      "asc");
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());

  while (q.next()) {
    QString name;
    name = q.value(0).toString() + q.value(1).toString();
    ExfoliatorNames << name;
  }
  return ExfoliatorNames;
}

QStringList QNode::SQLGetWaferNames() {
  QStringList WaferNames;
  QSqlQuery q("", db);

  q.prepare("SELECT wafer_structure from wafer order by id_wafer asc");
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());

  while (q.next()) {
    WaferNames << q.value(0).toString();
  }
  return WaferNames;
}

QStringList QNode::SQLGetCrystalNames() {
  QStringList CrystalNames;
  QSqlQuery q("", db);
  q.prepare(
      "select id_crystal, name_material_short, growth_lot"
      " from crystal inner join material"
      " on (crystal.id_crystal_material_fk = material.id_crystal_material)"
      " order by id_crystal_material");
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());

  while (q.next()) {
    CrystalNames << (q.value(0).toString() + "," + q.value(1).toString() + "," +
                     q.value(2).toString());
  }
  return CrystalNames;
}

int QNode::SQLGetMaximumChipID() {
  QSqlQuery q("", db);
  int id_chip;

  q.prepare("SELECT MAX(id_chip) FROM chip");
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
  q.next();
  id_chip = q.value(0).toInt();
  return id_chip;
}

int QNode::SQLGetMaximumSearchID(int id_chip) {
  QSqlQuery q("", db);
  int id_search;

  q.prepare("SELECT MAX(id_search) FROM search where id_chip_fk = ?");
  q.addBindValue(id_chip);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
  q.next();
  id_search = q.value(0).toInt();
  ROS_INFO("ID_SEARCH:%d", id_search);

  return id_search;
}

int QNode::SQLGetMaximumOriginID() {
  QSqlQuery q("", db);
  int id_origin_image;
  q.prepare("SELECT MAX(id_origin_image) FROM origin_images");
  q.exec();
  while (q.next()) {
    id_origin_image = q.value(0).toInt();
    printf("id_origin_image:%d\n", id_origin_image);
  }
  return id_origin_image;
}

int QNode::SQLGetChipID(int chiptray_id, int pos_in_chiptray) {
  QSqlQuery q("", db);
  int id_chip;

  q.prepare(
      "select id_chip from mydb.chip where id_chiptray_fk = :id_chiptray AND "
      "position_in_chiptray = :position_in_chiptray");
  q.bindValue(":id_chiptray", chiptray_id);
  q.bindValue(":id_position_in_chiptray", pos_in_chiptray);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());

  q.next();
  id_chip = q.value(0).toInt();
  ROS_INFO("ID_CHIP:%d", id_chip);
  return id_chip;
}

void QNode::SQLSetDatetimeSearchStarted(int id_search, int id_chip) {
  QSqlQuery q("", db);
  q.prepare(
      "UPDATE mydb.search SET datetime_search_started=CURRENT_TIMESTAMP "
      "WHERE id_search = ? and id_chip_fk = ? ");
  q.bindValue(0, id_search);
  q.bindValue(1, id_chip);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
}

void QNode::SQLSetDatetimeSearchFinished(int id_search, int id_chip) {
  QSqlQuery q("", db);
  q.prepare(
      "UPDATE mydb.search SET datetime_search_finished=CURRENT_TIMESTAMP "
      "WHERE id_search = ? and id_chip_fk = ? ");
  q.bindValue(0, id_search);
  q.bindValue(1, id_chip);
  if (!q.exec())
    ROS_INFO("QUERY ERROR, ERROR:%s SENT:%s",
             q.lastError().text().toStdString().c_str(),
             q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
}

void QNode::SQLInsertMetaImage(int flakecount, char* filename, int pos_x,
                               int pos_y, int id_search, int id_chip) {
  QSqlQuery q("", db);
  q.prepare(
      "INSERT INTO mydb.meta_images (id_meta_images, "
      "filename_meta_image, position_x, position_y, search_id_search, "
      "search_id_chip_fk, objective_lens_id_objective_lens, "
      "table_optical_filter_id_optical_filter) VALUES (?, ?, ?, ?, ?, "
      "?, ?, ?)");
  q.bindValue(0, flakecount);
  q.bindValue(1, filename);
  q.bindValue(2, pos_x);
  q.bindValue(3, pos_y);
  q.bindValue(4, id_search);
  q.bindValue(5, id_chip);
  q.bindValue(6, 3);
  q.bindValue(7, 0);
  if (!q.exec())
    ROS_INFO("QUERY ERROR, ERROR:%s SENT:%s",
             q.lastError().text().toStdString().c_str(),
             q.lastQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
}

void QNode::ExecuteAutofocus() {
  std_msgs::Empty emp;
  afc_5_publisher_sc0.publish(emp);
}

void QNode::SetObjectiveLensTartlet(int TartletPos) {
  std_msgs::UInt8 rev;
  std_msgs::UInt8 inten;
  const int inten_tuple[] = {70, 80, 130, 220, 250}; /// These values apply for 85nm and 290 nm SiO2/Si substrate
  //const int inten_tuple[] = {40, 50, 100, 160, 250}; /// These values apply for 200 nm SiO2/Si substrate

  rev.data = TartletPos + 1;
  lv_ncnt_n_publisher.publish(rev);
  afc_5_publisher.publish(rev);
  inten.data = inten_tuple[TartletPos];
  nic_100a_inten_publisher.publish(inten);
}

void QNode::ExtractPointListFromTableWidget(
    std::list<geometry_msgs::Point>* PointList, int row, int stepx, int stepy) {
  geometry_msgs::Point P[4];
  geometry_msgs::Point P_LeftBottom, P_RightTop;
  geometry_msgs::Point PointToSearch;
  QTableWidgetItem* item;

  PointList->clear();
  for (int i = 0; i < 4; i++) {
    item = SearchAreaTableWidget->item(row, i * 2);
    P[i].x = item->text().toDouble();
    item = SearchAreaTableWidget->item(row, i * 2 + 1);
    P[i].y = item->text().toDouble();
  }
  ////////////////////////////
  // Extract Surrounding Rectangle
  ////////////////////////////
  P_LeftBottom = P[0];
  for (int i = 1; i < 4; i++)
    if (P_LeftBottom.x > P[i].x) P_LeftBottom.x = P[i].x;
  for (int i = 1; i < 4; i++)
    if (P_LeftBottom.y > P[i].y) P_LeftBottom.y = P[i].y;

  P_RightTop = P[0];
  for (int i = 1; i < 4; i++)
    if (P_RightTop.x < P[i].x) P_RightTop.x = P[i].x;
  for (int i = 1; i < 4; i++)
    if (P_RightTop.y < P[i].y) P_RightTop.y = P[i].y;

  //////////////////////////
  // Extract Points to Search
  //////////////////////////
  int steppoint = ((P_RightTop.x - P_LeftBottom.x) / stepx *
                   (P_RightTop.y - P_LeftBottom.y) / stepy);
  PointToSearch = P_LeftBottom;

  for (int i = 0; i < steppoint; i++) {
    if (isInside(P, PointToSearch)) {
      PointList->push_back(PointToSearch);
    }
    PointToSearch.y = PointToSearch.y + stepy;
    if (PointToSearch.y > P_RightTop.y || PointToSearch.y < P_LeftBottom.y) {
      PointToSearch.y = PointToSearch.y - stepy;
      stepy = -stepy;
      PointToSearch.x = PointToSearch.x + stepx;
    }
  }
  return;
}

void QNode::on_Button_AddRecord_clicked() {
  int nofchips = spinBox_NumOfChips->value();
  int id_chiptray = spinBox_ChiptrayID_DataManip->value();
  int id_wafer = comboBox_Wafer->currentIndex();
  QDateTime datetime = dateEdit_Exfoliated->dateTime();
  QString exfoliated_place = comboBox_ExfoliatedPlace->currentText();
  int id_crystal = comboBox_Crystal->currentIndex();
  int id_exfoliator = comboBox_Exfoliator->currentIndex();

  QMessageBox msgBox;
  msgBox.setText(tr("Add New Chip Records to RDBMS."));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Cancel) {
    return;
  }

  SQLInsertChip(nofchips, id_chiptray, id_wafer,
                datetime, exfoliated_place,
                id_crystal, id_exfoliator);
}

void QNode::on_Button_Sequence_clicked() {
  on_Button_CaptureOrigin_clicked();
  on_Button_CaptureAlignmentImage_clicked();
  on_Button_StartFind_clicked();
}

void QNode::on_Button_CaptureAlignmentImage_clicked() {
  int flakecount = 0;
  std::list<geometry_msgs::Point> PointList;
  QTableWidgetItem* item;
  QTableWidgetItem* item_progress;
  int row;
  double delay;

  for (row = 0; row < SearchAreaTableWidget->rowCount(); row++) {
    ExtractPointListFromTableWidget(&PointList, row, spinBox_StepX->value() * 5,
                                    spinBox_StepY->value() * 5);
    std_srvs::Empty emp_srv;
    char filename[4028];
    double progress = 0.0;
    double count = 0;
    int id_chip;
    int id_search;

    SetObjectiveLensTartlet(0);
    ros::Duration(0.5).sleep();

    adm2_abs_publisher.publish(*PointList.begin());
    adm2_wait_for_stop_client.call(emp_srv);

    ExecuteAutofocus();
    ros::Duration(0.5).sleep();

    currentState = Search;
    item_progress = SearchAreaTableWidget->item(row, 9);

    id_chip =
        SQLGetChipID(static_cast<int>(spinBox_ChiptrayID->value()),
                     SearchAreaTableWidget->item(row, 10)->text().toInt());
    id_search = SQLGetMaximumSearchID(id_chip);

    for (std::list<geometry_msgs::Point>::iterator itr = PointList.begin();
         itr != PointList.end();) {
      adm2_abs_publisher.publish(*itr);

      progress = count / PointList.size();
      item_progress->setText(QString::number(progress * 100));

      delay = (static_cast<double>(spinBox_Delay->value())) / 100;
      ros::Duration(delay).sleep();
      count++;
      fp_tdmms_finder_Live();
      item = SearchAreaTableWidget->item(row, 8);
      snprintf(filename, sizeof(filename),
               "%s/align_%010d_%010d_%010d_%010d.jpg",
               item->text().toStdString().c_str(), flakecount,
               static_cast<int>(itr->x), static_cast<int>(itr->y), 0);
      fp_tdmms_finder_SaveImage(filename);
      SQLInsertAlignmentImage(filename, static_cast<int>(itr->x),
                              static_cast<int>(itr->y), id_search, id_chip);
      flakecount++;
      itr++;
      qApp->processEvents();
    }
  }
  currentState = Live;
}

void QNode::on_Button_StartFind_clicked() {
  geometry_msgs::Point PointToSearch;
  std::list<geometry_msgs::Point> PointList;
  QTableWidgetItem* item;
  QTableWidgetItem* item_progress;

  double delay;
  for (int row = 0; row < SearchAreaTableWidget->rowCount(); row++) {
    ExtractPointListFromTableWidget(&PointList, row, spinBox_StepX->value(),
                                    spinBox_StepY->value());
    std_msgs::UInt8 rev;
    std_msgs::UInt8 inten;
    std_srvs::Empty emp_srv;
    std_msgs::Empty emp;
    int flakecount = 0;
    char filename[4028];
    double progress = 0.0;
    double count = 0;
    QString str;
    int id_chip;
    int id_search;

    if (row == 0) {
      SetObjectiveLensTartlet(0);
      ros::Duration(0.5).sleep();
    }

    adm2_abs_publisher.publish(*PointList.begin());
    adm2_wait_for_stop_client.call(emp_srv);

    ExecuteAutofocus();
    ros::Duration(0.5).sleep();

    SetObjectiveLensTartlet(3);
    ros::Duration(5).sleep();

    ExecuteAutofocus();
    ros::Duration(1).sleep();

    ExecuteAutofocus();
    ros::Duration(1).sleep();

    currentState = Search;
    item_progress = SearchAreaTableWidget->item(row, 9);

    id_chip =
        SQLGetChipID(static_cast<int>(spinBox_ChiptrayID->value()),
                     SearchAreaTableWidget->item(row, 10)->text().toInt());
    id_search = SQLGetMaximumSearchID(id_chip);
    SQLSetDatetimeSearchStarted(id_search, id_chip);

    for (std::list<geometry_msgs::Point>::iterator itr = PointList.begin();
         itr != PointList.end();) {
      adm2_abs_publisher.publish(*itr);
      progress = count / PointList.size();
      str = QString::number(progress * 100);
      item_progress->setText(str);
      delay = (static_cast<double>(spinBox_Delay->value())) / 1000;
      ros::Duration(delay).sleep();
      count++;
      fp_tdmms_finder_Live();
      if (currentState == Search) {
        bool result;
        int pos_x, pos_y;
        unsigned int score;

        fp_tdmms_finder_Find(&result, &pos_x, &pos_y, &score);

        if (checkBox_SaveAll->isChecked()) {
          snprintf(
              filename, sizeof(filename), "%s/a_%010d_%010d_%010d_%010d.tiff",
              SearchAreaTableWidget->item(row, 8)->text().toStdString().c_str(),
              flakecount, static_cast<int>(itr->x), static_cast<int>(itr->y),
              score);
          fp_tdmms_finder_SaveImage_raw(filename);
          SQLInsertMetaImage(flakecount, filename, static_cast<int>(itr->x),
                             static_cast<int>(itr->y), id_search, id_chip);
          flakecount++;
        }

        if (result == true) {
          geometry_msgs::Point point_new;
          ObjectFeatures feature;
          point_new.x = itr->x + 120.0 / 2000.0 * (pos_x - 1000);
          point_new.y = itr->y - 120.0 / 2000.0 * (pos_y - 500);
          ROS_INFO("Flake Detected:%f, %f, %f, %f", itr->x, itr->y, point_new.x,
                   point_new.y);
          adm2_abs_publisher.publish(point_new);
          ros::Duration(delay + 0.2).sleep();
          fp_tdmms_finder_Live();
          fp_tdmms_finder_Find(&result, &pos_x, &pos_y, &score);
          if (result == true) {
            if (!SQLCheckDuplicate(id_search, id_chip, static_cast<int>(point_new.x),
                                   static_cast<int>(point_new.y), 20)) {
              snprintf(filename, sizeof(filename),
                       "%s/%010d_%010d_%010d_%010d.jpg",
                       SearchAreaTableWidget->item(row, 8)
                           ->text()
                           .toStdString()
                           .c_str(),
                       flakecount, static_cast<int>(point_new.x),
                       static_cast<int>(point_new.y), score);
              fp_tdmms_finder_SaveImage(filename);
              fp_tdmms_finder_ExtractFeatures(&feature);

              SQLInsertFlakeImage(
                  flakecount, 3, filename, static_cast<int>(point_new.x),
                  static_cast<int>(point_new.y), id_search, id_chip, feature);
              flakecount++;

              snprintf(filename, sizeof(filename),
                       "%s/r_%010d_%010d_%010d_%010d.jpg",
                       SearchAreaTableWidget->item(row, 8)
                           ->text()
                           .toStdString()
                           .c_str(),
                       flakecount, static_cast<int>(point_new.x),
                       static_cast<int>(point_new.y), score);
              fp_tdmms_finder_SaveRegion(filename);

              memset(&feature, 0x00, sizeof(feature));
              feature.area = score;
              SQLInsertFlakeImage(
                  flakecount, 5, filename, static_cast<int>(point_new.x),
                  static_cast<int>(point_new.y), id_search, id_chip, feature);
              flakecount++;
            }
          }
        }
        itr++;
      }
      qApp->processEvents();
    }

    if (!checkBox_simplemode->isChecked()) {
      for (int revo = 2; revo >= 0; revo--) {
        QSqlQuery q("", db);
        SetObjectiveLensTartlet(revo);
        q.prepare(
            "select position_x, position_y, area from mydb.flake_images where "
            "search_id_search = ? and search_id_chip_fk = ? "
            "and objective_lens_id_objective_lens_fk = 3");
        q.addBindValue(id_search);
        q.addBindValue(id_chip);
        if (!q.exec())
          ROS_WARN("QUERY ERROR, ERROR:%s SENT:%s",
                   q.lastError().text().toStdString().c_str(),
                   q.executedQuery().toStdString().c_str());
        else
          ROS_DEBUG("QUERY SUCCESS, SENT:%s",
                    q.executedQuery().toStdString().c_str());
        ROS_INFO("ID_Search: %d", id_search);

        while (q.next()) {
          int score;
          ObjectFeatures feature;
          PointToSearch.x = q.value(0).toInt();
          PointToSearch.y = q.value(1).toInt();
          score = q.value(2).toInt();
          adm2_abs_publisher.publish(PointToSearch);
          adm2_wait_for_stop_client.call(emp_srv);

          afc_5_publisher_sc0.publish(emp);
          ros::Duration(2).sleep();

          fp_tdmms_finder_Live();
          item = SearchAreaTableWidget->item(row, 8);
          snprintf(filename, sizeof(filename), "%s/%010d_%010d_%010d_%010d.jpg",
                   item->text().toStdString().c_str(), flakecount,
                   static_cast<int>(PointToSearch.x),
                   static_cast<int>(PointToSearch.y), score);
          fp_tdmms_finder_SaveImage(filename);

          memset(&feature, 0x00, sizeof(feature));
          feature.area = score;
          SQLInsertFlakeImage(
              flakecount, revo, filename, static_cast<int>(PointToSearch.x),
              static_cast<int>(PointToSearch.y), id_search, id_chip, feature);
          flakecount++;
          qApp->processEvents();
        }
      }
    }
    SQLSetDatetimeSearchFinished(id_search, id_chip);
  }
  currentState = Live;
}

bool QNode::isInside(geometry_msgs::Point rect[4], geometry_msgs::Point pnt) {
  //////////////////////////////////////////////////////////
  //// returns true if pnt is inside the rectangle defined by rect
  ///////////////////////////////////////////////////////////

  int cnt = 0;
  int i;

  for (i = 0; i < 4; ++i) {
    const double x1 = rect[(i + 1) % 4].x - rect[i].x;
    const double y1 = rect[(i + 1) % 4].y - rect[i].y;
    const double x2 = pnt.x - rect[i].x;
    const double y2 = pnt.y - rect[i].y;
    if (x1 * y2 - x2 * y1 < 0) {
      ++cnt;
    } else {
      --cnt;
    }
  }
  return cnt == 4 || cnt == -4;
}

void QNode::on_Button_Add_clicked() {
  SearchAreaTableWidget->insertRow(SearchAreaTableWidget->rowCount());
  SearchAreaTableWidget->resizeColumnsToContents();
  SearchAreaTableWidget->resizeRowsToContents();
  int i;
  QTableWidgetItem* item;
  for (i = 0; i < 12; i++) {
    item = new QTableWidgetItem(QString("0"));
    SearchAreaTableWidget->setItem(SearchAreaTableWidget->rowCount() - 1, i,
                                   item);
  }
  SearchAreaTableWidget->setCurrentItem(item);
}

void QNode::on_Button_AutoSearchImage_clicked() {
  double delay = 0.3;
  char filename[4092];
  int i, j;
  geometry_msgs::Point movpoint;
  std_msgs::UInt8 rev;
  std_msgs::UInt8 inten;
  std_srvs::Empty emp_srv;

  //
  SetObjectiveLensTartlet(0);
  ros::Duration(1).sleep();

  adm2::velocity vel;
  vel.velocity_low = 500;
  vel.velocity_high = 10000;
  vel.accl = 100;
  adm2_vel_publisher.publish(vel);

  ros::Duration(0.1).sleep();
  adm2_abs_publisher.publish(startpoint);
  adm2_wait_for_stop_client.call(emp_srv);

  currentState = Search;

  const char *homedir;

  if((homedir = getenv("HOME"))==NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  for (i = 0; i < syasinnnokazu_tate; i++) {
    for (j = 0; j < syasinnnokazu_yoko; j++) {
      fp_tdmms_finder_Live();
      snprintf(filename, sizeof(filename),
               "%s/images/temporary_images/%d.tif",
               homedir,
               j + i * syasinnnokazu_yoko);
      fp_tdmms_finder_SaveImage_affine(filename);
      movpoint.x = yokoidou;
      movpoint.y = 0;
      adm2_stp_publisher.publish(movpoint);
      ros::Duration(delay).sleep();
      qApp->processEvents();
    }
    ros::Duration(delay).sleep();
    movpoint.x = 0;
    movpoint.y = tateidou;
    adm2_stp_publisher.publish(movpoint);
    ros::Duration(delay).sleep();
    yokoidou = -yokoidou;
  }
  currentState = Live;
}

void QNode::on_Button_AutoSearchArea_clicked() {
  int i, j;
  int numchip;
  int pos_x1[256], pos_x2[256], pos_x3[256], pos_x4[256];
  int pos_y1[256], pos_y2[256], pos_y3[256], pos_y4[256];
  int pos_in_chiptray[36];
  int chipid[36];

  double scaling_factor = (static_cast<double>(yokoidou)) / 2034.0 * 8.0;
  double dbuf;

  // Local iconic variables
  HObject ho_GrayImage_list, ho_Image, ho_TiledImage, ho_Region1;
  HObject ho_ConnectedRegions, ho_Circle1, ho_Circle2;
  HObject ho_Circle3, ho_Circle4, ho_Circles;
  HObject ho_ConnectedRegions_sort;

  // Local control variables
  HTuple hv_exposure, hv_yokoidou, hv_tateidou;
  HTuple hv_temporary_folder, hv_Tuple11;
  HTuple hv_Tuple_row, hv_Tuple_col, hv_Index, hv_Row, hv_Column;
  HTuple hv_Phi, hv_Length1, hv_Length2, hv_ColPoint1, hv_ColPoint2;
  HTuple hv_ColPoint3, hv_ColPoint4, hv_RowPoint1, hv_RowPoint2;
  HTuple hv_RowPoint3, hv_RowPoint4, hv_count, hv_radius;

  // Initializing Parameters
  HTuple hv_homedir;

  TupleEnvironment("HOME", &hv_homedir);
  hv_temporary_folder = hv_homedir + "/images/temporary_images/";

  // Tile Image
  for (i = 0; i < syasinnnokazu_yoko * syasinnnokazu_tate; i++)
    hv_Tuple11[i] = -1;

  for (i = 0; i < syasinnnokazu_tate; i++) {
    for (j = 0; j < syasinnnokazu_yoko; j++) {
      hv_Tuple_row[j + (i * syasinnnokazu_yoko)] = (i * 1016) / 8;
      if ((i % 2) == 0) {
        hv_Tuple_col[j + (i * syasinnnokazu_yoko)] = (j * 2034) / 8;
      } else {
        hv_Tuple_col[j + (i * syasinnnokazu_yoko)] =
            ((syasinnnokazu_yoko - j) * 2034) / 8;
      }
    }
  }

  GenEmptyObj(&ho_GrayImage_list);
  for (i = syasinnnokazu_yoko * syasinnnokazu_tate - 1; i >= 0; i--) {
    hv_Index = i;
    ReadImage(&ho_Image, (hv_temporary_folder + hv_Index) + ".tif");
    ConcatObj(ho_Image, ho_GrayImage_list, &ho_GrayImage_list);
  }

  TileImagesOffset(ho_GrayImage_list, &ho_TiledImage, hv_Tuple_row,
                   hv_Tuple_col, hv_Tuple11, hv_Tuple11, hv_Tuple11, hv_Tuple11,
                   10000, 10000);

  /// Threshold Tiled Image and Extract Chip Regions
  Threshold(ho_TiledImage, &ho_Region1, 50, 255);
  ClosingCircle(ho_Region1, &ho_Region1, 30);
  ErosionCircle(ho_Region1, &ho_Region1, 180.5);
  Connection(ho_Region1, &ho_ConnectedRegions);
  SortRegion(ho_ConnectedRegions, &ho_ConnectedRegions_sort, "character",
             "true", "row");

  /*
    // Debug code --- Display the results of color thresholding: ho_Region1
  HTuple hv_WindowHandle_originPos;
  HalconCpp::OpenWindow(0, 744 * 1.5, 744 * 1.5, 480 * 1.5, 0, "", "",
                        &hv_WindowHandle_originPos);
  HalconCpp::HDevWindowStack::Push(hv_WindowHandle_originPos);
  HDevWindowStack::SetActive(hv_WindowHandle_originPos);
  DispObj(ho_Region1, HDevWindowStack::GetActive());

  QMessageBox msgBox;
  msgBox.setText(tr("Capture Origin Images at designated positions."));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  msgBox.exec();
  HDevWindowStack::SetActive(hv_WindowHandle_originPos);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
  */
  
  SmallestRectangle2(ho_ConnectedRegions_sort, &hv_Row, &hv_Column, &hv_Phi,
                     &hv_Length1, &hv_Length2);
  hv_Phi = -hv_Phi;

  hv_ColPoint1 = (hv_Column + (hv_Length1 * (hv_Phi.TupleCos()))) -
                 (hv_Length2 * (hv_Phi.TupleSin()));
  hv_ColPoint2 = (hv_Column + (hv_Length1 * (hv_Phi.TupleCos()))) +
                 (hv_Length2 * (hv_Phi.TupleSin()));
  hv_ColPoint3 = (hv_Column - (hv_Length1 * (hv_Phi.TupleCos()))) -
                 (hv_Length2 * (hv_Phi.TupleSin()));
  hv_ColPoint4 = (hv_Column - (hv_Length1 * (hv_Phi.TupleCos()))) +
                 (hv_Length2 * (hv_Phi.TupleSin()));
  hv_RowPoint1 = (hv_Row + (hv_Length1 * (hv_Phi.TupleSin()))) +
                 (hv_Length2 * (hv_Phi.TupleCos()));
  hv_RowPoint2 = (hv_Row + (hv_Length1 * (hv_Phi.TupleSin()))) -
                 (hv_Length2 * (hv_Phi.TupleCos()));
  hv_RowPoint3 = (hv_Row - (hv_Length1 * (hv_Phi.TupleSin()))) +
                 (hv_Length2 * (hv_Phi.TupleCos()));
  hv_RowPoint4 = (hv_Row - (hv_Length1 * (hv_Phi.TupleSin()))) -
                 (hv_Length2 * (hv_Phi.TupleCos()));

  CountObj(ho_ConnectedRegions, &hv_count);
  /*TupleGenConst(hv_count, 60, &hv_radius);
  GenCircle(&ho_Circle1, hv_RowPoint1, hv_ColPoint1, hv_radius);
  GenCircle(&ho_Circle2, hv_RowPoint2, hv_ColPoint2, hv_radius);
  GenCircle(&ho_Circle3, hv_RowPoint3, hv_ColPoint3, hv_radius);
  GenCircle(&ho_Circle4, hv_RowPoint4, hv_ColPoint4, hv_radius);
  ConcatObj(ho_Circle1, ho_Circle2, &ho_Circles);
  ConcatObj(ho_Circles, ho_Circle3, &ho_Circles);
  ConcatObj(ho_Circles, ho_Circle4, &ho_Circles);
  */
  numchip = static_cast<int>(hv_count);

  for (i = 0; i < numchip; i++) {
    dbuf = hv_ColPoint1[i];
    pos_x1[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(yokoidou / 2) + startpoint.x;
    dbuf = hv_ColPoint2[i];
    pos_x2[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(yokoidou / 2) + startpoint.x;
    dbuf = hv_ColPoint3[i];
    pos_x4[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(yokoidou / 2) + startpoint.x;
    dbuf = hv_ColPoint4[i];
    pos_x3[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(yokoidou / 2) + startpoint.x;

    dbuf = hv_RowPoint1[i];
    pos_y1[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(tateidou / 2) + startpoint.y;
    dbuf = hv_RowPoint2[i];
    pos_y2[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(tateidou / 2) + startpoint.y;
    dbuf = hv_RowPoint3[i];
    pos_y4[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(tateidou / 2) + startpoint.y;
    dbuf = hv_RowPoint4[i];
    pos_y3[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(tateidou / 2) + startpoint.y;

    int pos_center_x =
        static_cast<int>((pos_x1[i] + pos_x2[i] + pos_x3[i] + pos_x4[i]) / 4);
    int pos_center_y =
        static_cast<int>((pos_y1[i] + pos_y2[i] + pos_y3[i] + pos_y4[i]) / 4);
    pos_in_chiptray[i] = static_cast<int>((pos_center_x - (-23966)) / 7000) -
                         6 * static_cast<int>((pos_center_y - (24000)) / 7000) +
                         1;

    // Get Chip ID
    chipid[i] = SQLGetChipID(static_cast<int>(spinBox_ChiptrayID->value()),
                             pos_in_chiptray[i]);
  }

  for (i = 0; i < numchip; i++) {
    on_Button_Add_clicked();
    SearchAreaTableWidget->item(i, 0)->setText(QString::number(pos_x1[i]));
    SearchAreaTableWidget->item(i, 1)->setText(QString::number(pos_y1[i]));
    SearchAreaTableWidget->item(i, 2)->setText(QString::number(pos_x2[i]));
    SearchAreaTableWidget->item(i, 3)->setText(QString::number(pos_y2[i]));
    SearchAreaTableWidget->item(i, 4)->setText(QString::number(pos_x3[i]));
    SearchAreaTableWidget->item(i, 5)->setText(QString::number(pos_y3[i]));
    SearchAreaTableWidget->item(i, 6)->setText(QString::number(pos_x4[i]));
    SearchAreaTableWidget->item(i, 7)->setText(QString::number(pos_y4[i]));
    SearchAreaTableWidget->item(i, 10)
        ->setText(QString::number(pos_in_chiptray[i]));
    SearchAreaTableWidget->item(i, 11)->setText(QString::number(chipid[i]));
  }
}

void QNode::on_Button_CaptureOrigin_clicked() {
  int i, j;

  // Local iconic variables
  HObject ho_GrayImage_list, ho_Image, ho_TiledImage;
  HObject ho_Region1, ho_BinImage, ho_ImageCorner, ho_Corners;
  HObject ho_CornersConnected, ho_Region1Connected, ho_Region1Connected_sort;
  HObject ho_CornersConnected_sort, ho_Corner, ho_SelectedChip;
  HObject ho_chip_temp;

  // Local control variables
  HTuple hv_yokoidou, hv_tateidou;
  HTuple hv_temporary_folder;
  HTuple hv_Tuple11, hv_i, hv_j, hv_Tuple_row, hv_Tuple_col;
  HTuple hv_Index, hv_row, hv_column, hv_radius, hv_count;
  HTuple hv_chipcount, hv_col_c, hv_row_c, hv_isEqual, hv_chipno;

  double pos_x[4096], pos_y[4096];

  SetObjectiveLensTartlet(0);
  ros::Duration(1).sleep();

  hv_yokoidou = 1120;
  hv_tateidou = -560;
  HTuple hv_homedir;

  TupleEnvironment("HOME", &hv_homedir);
  hv_temporary_folder = hv_homedir + "/images/temporary_images/";

  for (i = 0; i < syasinnnokazu_yoko * syasinnnokazu_tate; i++) {
    hv_Tuple11[i] = -1;
  }

  GenEmptyObj(&ho_GrayImage_list);

  for (i = 0; i < syasinnnokazu_tate; i++) {
    for (j = 0; j < syasinnnokazu_yoko; j++) {
      hv_Tuple_row[j + (i * syasinnnokazu_yoko)] = (i * 1016) / 8;
      if ((i % 2) == 0) {
        hv_Tuple_col[j + (i * syasinnnokazu_yoko)] = (j * 2034) / 8;
      } else {
        hv_Tuple_col[j + (i * syasinnnokazu_yoko)] =
            ((syasinnnokazu_yoko - j) * 2034) / 8;
      }
    }
  }

  for (hv_Index = (syasinnnokazu_yoko * syasinnnokazu_tate) - 1; hv_Index >= 0;
       hv_Index += -1) {
    ReadImage(&ho_Image, (hv_temporary_folder + hv_Index) + ".tif");
    ConcatObj(ho_Image, ho_GrayImage_list, &ho_GrayImage_list);
    qApp->processEvents();
  }

  TileImagesOffset(ho_GrayImage_list, &ho_TiledImage, hv_Tuple_row,
                   hv_Tuple_col, hv_Tuple11, hv_Tuple11, hv_Tuple11, hv_Tuple11,
                   10000, 10000);
  Threshold(ho_TiledImage, &ho_Region1, 50, 200);
  ClosingRectangle1(ho_Region1, &ho_Region1, 50, 50);
  ClosingCircle(ho_Region1, &ho_Region1, 50);
  ErosionCircle(ho_Region1, &ho_Region1, 10);
  RegionToBin(ho_Region1, &ho_BinImage, 255, 0, 10000, 10000);
  CornerResponse(
      ho_BinImage, &ho_ImageCorner,
      static_cast<int>(
          comboBox_cornerMask
              ->itemData(static_cast<int>(comboBox_cornerMask->currentIndex()))
              .toInt()),
      static_cast<double>(doubleSpinBox_cornerWeight->value()));
  Threshold(ho_ImageCorner, &ho_Corners, 50, 255);
  Connection(ho_Corners, &ho_CornersConnected);
  DilationCircle(ho_CornersConnected, &ho_CornersConnected, 100);

  HTuple hv_WindowHandle_originPos;
  HalconCpp::OpenWindow(0, 744 * 1.5, 744 * 1.5, 480 * 1.5, 0, "", "",
                        &hv_WindowHandle_originPos);
  HalconCpp::HDevWindowStack::Push(hv_WindowHandle_originPos);

  HDevWindowStack::SetActive(hv_WindowHandle_originPos);
  if (HDevWindowStack::IsOpen())
    DispObj(ho_TiledImage, HDevWindowStack::GetActive());
  if (HDevWindowStack::IsOpen())
    DispObj(ho_CornersConnected, HDevWindowStack::GetActive());

  QMessageBox msgBox;
  msgBox.setText(tr("Capture Origin Images at designated positions."));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Cancel) {
    HDevWindowStack::SetActive(hv_WindowHandle_originPos);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    return;
  }

  HDevWindowStack::SetActive(hv_WindowHandle_originPos);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  Connection(ho_Region1, &ho_Region1Connected);
  SortRegion(ho_Region1Connected, &ho_Region1Connected_sort, "character",
             "true", "row");
  SortRegion(ho_CornersConnected, &ho_CornersConnected_sort, "character",
             "true", "row");
  SmallestCircle(ho_CornersConnected_sort, &hv_row, &hv_column, &hv_radius);

  CountObj(ho_CornersConnected_sort, &hv_count);
  CountObj(ho_Region1Connected_sort, &hv_chipcount);

  for (i = 0; i < hv_count; i++) {
    hv_col_c = ((const HTuple&)hv_column)[i];
    hv_row_c = ((const HTuple&)hv_row)[i];
    SelectObj(ho_CornersConnected_sort, &ho_Corner, i + 1);
    SelectShapeProto(ho_Region1Connected_sort, ho_Corner, &ho_SelectedChip,
                     "distance_contour", 0, 100);
    for (j = 0; j < hv_chipcount; j++) {
      HObject ho_inter, ho_inter_con;
      HTuple hv_area, hv_ro, hv_co;

      SelectObj(ho_Region1Connected_sort, &ho_chip_temp, j + 1);
      Intersection(ho_chip_temp, ho_SelectedChip, &ho_inter);
      Connection(ho_inter, &ho_inter_con);
      AreaCenter(ho_inter, &hv_area, &hv_ro, &hv_co);

      if (hv_area != 0) {
        hv_chipno[i] = j + 1;
      }
      qApp->processEvents();
    }
  }

  int numcorner = static_cast<int>(hv_count);
  double scaling_factor = (static_cast<double>(yokoidou)) / 2034.0 * 8.0;
  double dbuf;

  for (i = 0; i < static_cast<int>(hv_chipcount); i++) {
    int id_chip =
        SQLGetChipID(static_cast<int>(spinBox_ChiptrayID->value()),
                     SearchAreaTableWidget->item(i, 10)->text().toInt());
    int id_search = SQLGetMaximumSearchID(id_chip);
    id_search++;

    long target_layer_num_min, target_layer_num_max;
    Hcpar param;
    char c_para[] = "target_layer_num_min";
    fp_tdmms_finder_GetParameter(c_para, &param);
    target_layer_num_min = param.par.l;

    char c_para_2[] = "target_layer_num_max";
    fp_tdmms_finder_GetParameter(c_para_2, &param);
    target_layer_num_max = param.par.l;

    SQLInsertSearch(
        id_search, id_chip,
        listLibraryFile[static_cast<int>(FinderLibrarycomboBox->currentIndex())]
            .absoluteFilePath()
            .toAscii()
            .constData(),
        static_cast<int>(target_layer_num_min),
        static_cast<int>(target_layer_num_max));
  }

  for (i = 0; i < numcorner; i++) {
    dbuf = hv_column[i];
    pos_x[i] = static_cast<int>(dbuf * scaling_factor) -
               static_cast<int>(yokoidou / 2) + startpoint.x;
    dbuf = hv_row[i];
    pos_y[i] = -static_cast<int>(dbuf * scaling_factor) +
               static_cast<int>(tateidou / 2) + startpoint.y + 500;
  }

  currentState = Search;
  for (i = 0; i < numcorner; i++) {
    char filename[4096];
    QTableWidgetItem* item;
    std_msgs::Empty emp;
    std_srvs::Empty emp_srv;
    geometry_msgs::Point movpoint;

    movpoint.x = pos_x[i];
    movpoint.y = pos_y[i];
    adm2_abs_publisher.publish(movpoint);
    adm2_wait_for_stop_client.call(emp_srv);

    ros::Duration(1).sleep();
    afc_5_publisher_sc0.publish(emp);
    ros::Duration(1).sleep();
    ROS_DEBUG("chipno:%d\n", static_cast<int>(hv_chipno[i]));
    item = SearchAreaTableWidget->item(static_cast<int>(hv_chipno[i]) - 1, 8);

    sprintf(filename,"%s/o_%010d_%010d_%010d_%010d.jpg",
             item->text().toStdString().c_str(), i, static_cast<int>(pos_x[i]),
             static_cast<int>(pos_y[i]), static_cast<int>(0));

    ROS_DEBUG("%s\n", filename);
    ros::Duration(1).sleep();
    fp_tdmms_finder_Live();
    fp_tdmms_finder_SaveImage(filename);

    int id_search = SQLGetMaximumSearchID(
        SearchAreaTableWidget->item(static_cast<int>(hv_chipno[i]) - 1, 11)
            ->text()
            .toInt());

    int id_origin_image = SQLGetMaximumOriginID();
    id_origin_image++;
    SQLInsertOriginImage(
        id_origin_image, id_search,
        SearchAreaTableWidget->item(static_cast<int>(hv_chipno[i]) - 1, 11)
            ->text()
            .toInt(),
        static_cast<int>(pos_x[i]), static_cast<int>(pos_y[i]), filename);
    qApp->processEvents();
  }
  currentState = Live;
}

void QNode::on_Button_SetPicFolder_clicked() {
  QString strDir = QFileDialog::getExistingDirectory(
      0, tr("Folder to Store Image"), strDefaultImageFolder,
      QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly);
  if (!strDir.isEmpty()) {
    lineEdit_picFolder->setText(strDir);
  }
}

void QNode::on_Exposure_SpinBox_valueChanged() {
  long exposureTime;
  Hcpar param;
  char c_para[] = "exposure";

  exposureTime = spinBox_Exposure->value();
  param.par.l = exposureTime;
  fp_tdmms_finder_SetParameter(c_para, &param);
}

void QNode::on_Button_ClearAll_clicked() {}

void QNode::on_Button_Delete_clicked() {
  int row = SearchAreaTableWidget->currentRow();
  SearchAreaTableWidget->setCurrentIndex(
      SearchAreaTableWidget->model()->index(row, 0));
  SearchAreaTableWidget->removeRow(row);
}

void QNode::on_Button_HaltFind_clicked() {
  if (currentState == Search) {
    currentState = Halt;
  } else if (currentState == Halt) {
    currentState = Search;
  }
}

void QNode::on_Button_ResumeFind_clicked() {
}

void QNode::on_Button_SetFolderAll_clicked() {
  QString strDir = QFileDialog::getExistingDirectory(
      0, tr("Folder to Save Image"), strDefaultImageFolder,
      QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly);

  if (!strDir.isEmpty()) {
    for (int row = 0; row < SearchAreaTableWidget->rowCount(); row++) {
      QString nostr = strDir + QString("/%1").arg(row + 1, 2, 10, QChar('0'));
      SearchAreaTableWidget->item(row, 8)->setText(nostr);
      if (!QDir(nostr).exists()) QDir().mkdir(nostr);
    }
  }
  SearchAreaTableWidget->resizeColumnsToContents();
  SearchAreaTableWidget->resizeRowsToContents();
}

void QNode::on_Button_SetFolder_clicked() {
  QString strDir = QFileDialog::getExistingDirectory(
      0, tr("Save Image To"), strDefaultImageFolder,
      QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly);

  if (!strDir.isEmpty()) {
    int row = SearchAreaTableWidget->currentRow();
    SearchAreaTableWidget->item(row, 8)->setText(strDir);
  }
  SearchAreaTableWidget->resizeColumnsToContents();
  SearchAreaTableWidget->resizeRowsToContents();
}

void QNode::on_Button_SetP1_clicked() {
  int row = SearchAreaTableWidget->currentRow();
  SearchAreaTableWidget->item(row, 0)->setText(QString::number(currpoint.x));
  SearchAreaTableWidget->item(row, 1)->setText(QString::number(currpoint.y));
}

void QNode::on_Button_SetP2_clicked() {
  int row = SearchAreaTableWidget->currentRow();
  SearchAreaTableWidget->item(row, 2)->setText(QString::number(currpoint.x));
  SearchAreaTableWidget->item(row, 3)->setText(QString::number(currpoint.y));
}

void QNode::on_Button_SetP3_clicked() {
  int row = SearchAreaTableWidget->currentRow();
  SearchAreaTableWidget->item(row, 4)->setText(QString::number(currpoint.x));
  SearchAreaTableWidget->item(row, 5)->setText(QString::number(currpoint.y));
}

void QNode::on_Button_SetP4_clicked() {
  int row = SearchAreaTableWidget->currentRow();
  SearchAreaTableWidget->item(row, 6)->setText(QString::number(currpoint.x));
  SearchAreaTableWidget->item(row, 7)->setText(QString::number(currpoint.y));
}

}  // namespace tdmms_finder_auto
