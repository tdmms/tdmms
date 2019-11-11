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

#include <halcon_bridge/halcon_bridge.h>
#include <QTextStream>
#include <QFile>
#include <QString>
#include <QDir>
#include <QtSql/QtSql>
#include <QMessageBox>
#include <QFileDialog>
#include <QtGui>

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <adm2/velocity.h>
#include <adm2/GetCurrentPos.h>
#include <tdmms_finder_network_support/UploadS3.h>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <vector>
#include "../include/tdmms_finder_dl/qnode.hpp"
#include <pwd.h>

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

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_finder_dl {
using namespace Qt;

/*****************************************************************************
** Implementation
*****************************************************************************/

//////////////////////////////////////////////////////////////////////////////
//// QNode Constructor
//////////////////////////////////////////////////////////////////////////////
QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv) {
  ///////////////////////////////////////////////
  //// Parameters for optical microscope
  ///////////////////////////////////////////////
  step_x = 1120;
  step_y = -560;
  numsteps_x = 39;
  numsteps_y = 78;
  origin.x = -25000;
  origin.y = 25000;
  strDefaultImageFolder = QDir::homePath() + "/tdmms_data/images";
  strDefaultTemporaryFolder = QDir::homePath() + "/images/temporary_images";
  temp_filename = "/ramdisk/tmp.jpg";
  currentState = Live;
  hv_toolbar_offset = 60;
  issettingMode = false;
  ///////////////////////////////////////////////
  ///// Connect to MYSQL Server
  ///////////////////////////////////////////////
  QSqlError err;
  db = QSqlDatabase::addDatabase(QString("QMYSQL"), QString("Browser"));
  db.setDatabaseName(QString("2dmms_db"));
  db.setHostName(QProcessEnvironment::systemEnvironment().value("AWS_DB_URL","default"));
  db.setPort(3306);
  if (!db.open(QProcessEnvironment::systemEnvironment().value("AWS_DB_USER","default"),
               QProcessEnvironment::systemEnvironment().value("AWS_DB_PW","default"))) {
    err = db.lastError();
    db = QSqlDatabase();
    QSqlDatabase::removeDatabase(QString("Browser"));
  } else {
    ROS_INFO("connected to SQL Server");
  }
  p_query = new QSqlQuery(db);
  
  ///////////////////////////////////////////////
  /// Activate GPU Processing
  //////////////////////////////////////////////
  /// Do not activate GPU because it causes runtime error in write image
  /*
  HTuple hv_Index;

  QueryAvailableComputeDevices(&hv_DeviceIdentifiers);
  {
    HTuple end_val3 = (hv_DeviceIdentifiers.TupleLength()) - 1;
    HTuple step_val3 = 1;
    for (hv_Index = 0; hv_Index.Continue(end_val3, step_val3);
         hv_Index += step_val3) {
      GetComputeDeviceInfo(HTuple(hv_DeviceIdentifiers[hv_Index]), "name",
                           &hv_DeviceName);
      GetComputeDeviceInfo(HTuple(hv_DeviceIdentifiers[hv_Index]), "vendor",
                           &hv_DeviceVendor);
      OpenComputeDevice(HTuple(hv_DeviceIdentifiers[hv_Index]),
                        &hv_DeviceHandle);
    }
  }
  ActivateComputeDevice(hv_DeviceHandle);
  */
  /////////////////////////////////////////
  // Open window for optical microscope
  /////////////////////////////////////////
  TupleEnvironment("HOME", &hv_homedir);
  ReadImage(&ho_OMImage,
            hv_homedir +
            "/tdmms_ws/src/tdmms_stamper/tdmms_autostamp/tdmms_autostamp_master/resources/images/default_OM_image.jpg");
  GetImageSize(ho_OMImage, &hv_Width, &hv_Height);
  hv_windowScale = 1.0;

  ////////////////////////////////////////
  /// Prepare Background image
  ///////////////////////////////////////
  prepareBGImage(QDir::homePath() + "/images/masubuchi/Background_Image/SiO2_290nm_20190610_12bit.tiff");

  //////////////////////////////////////
  //  Window Open
  //////////////////////////////////////
  SetWindowAttr("window_title", "Preview Image");
  OpenWindow(0, 0, hv_Width * hv_windowScale, hv_Height * hv_windowScale, 0, "",
             "", &hv_WindowHandleOM);
  HDevWindowStack::Push(hv_WindowHandleOM);

  //////////////////////////////////////
  // Open Framegrabber (12 bit depth)
  //////////////////////////////////////
  hv_AcqName = "Pixci";
  OpenFramegrabber(hv_AcqName, 1, 1, 0, 0, 0, 0, "default", 12, "default", -1,
                   "false", "default", "default", -1, -1, &hv_AcqHandle);
  hv_exposure = 5000;
  SetFramegrabberParam(hv_AcqHandle, "exposure", hv_exposure);
}

QNode::~QNode() {
  delete p_query;
  db.close();
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
  CloseFramegrabber(hv_AcqHandle);
  CloseWindow(hv_WindowHandleOM);
}

void QNode::SQLInsertSearch(int id_search,
                            int id_chip,
                            const char* search_program,
                            int target_layer_num_min,
                            int target_layer_num_max) {
  ////////////////////////////////////////////////////////////
  /// Insert search record to DB
  ////////////////////////////////////////////////////////////
  /// int id_search: search id
  /// int id_chip: chip id
  /// const char* search program: the name of 2D materials detection program
  /// int target_layer_num_min: the minimum target thickness of flakes
  /// int target_layer_num_max: the maximum target thickness of flakes
  ////////////////////////////////////////////////////////////
  QSqlQuery q("", db);
  q.prepare(
      "INSERT INTO 2dmms_db.search (id_search, id_chip_fk, "
      "datetime_search_started, search_program, target_layer_num_min, "
      "target_layer_num_max ) VALUES (?, ?, CURRENT_TIMESTAMP, ?, ?, ?)");
  q.bindValue(0, id_search);
  q.bindValue(1, id_chip);
  q.bindValue(2, search_program);
  q.bindValue(3, target_layer_num_min);
  q.bindValue(4, target_layer_num_max);
  SQLExecQuery(&q);
}

bool QNode::SQLExecQuery(QSqlQuery *q) {
  ////////////////////////////////////////////////////
  //// Common function to execute SQL query
  ////////////////////////////////////////////////////
  //// QSqlQuery *q : query to be executed
  ////////////////////////////////////////////////////
  if (!q->exec()) {
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q->lastError().text().toStdString().c_str(),
              q->executedQuery().toStdString().c_str());
    return false;
  } else {
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q->executedQuery().toStdString().c_str());
    return true;
  }
}

int QNode::SQLGetNumOfChips(int id_chiptray) {
  ///////////////////////////////////////////////////////
  /// returns the number of silicon chips on the designated chiptray
  ///////////////////////////////////////////////////////
  /// int id_chiptray: chiptray id
  ///////////////////////////////////////////////////////
  QSqlQuery q("", db);
  q.prepare(
      "SELECT COUNT(idchip) from 2dmms_db.chip where idchiptray_fk = ?");
  q.addBindValue(id_chiptray);
  SQLExecQuery(&q);

  q.next();
  int numofchips = q.value(0).toInt();
  ROS_INFO("NumOfChips:%d", numofchips);
  return numofchips;
}

int QNode::SQLGetChipID(int chiptray_id,
                        int pos_in_chiptray) {
  ////////////////////////////////////////////////////////
  /// returns silicon chip id
  ////////////////////////////////////////////////////////
  /// int chiptray_id: chiptray id
  /// int pos_in_chiptray: positoin in chiptray (pocket no)
  ////////////////////////////////////////////////////////
  QSqlQuery q("", db);
  int id_chip;
  q.prepare(
      "select idchip from 2dmms_db.chip where idchiptray_fk = :idchiptray AND "
      "position_in_chiptray = :position_in_chiptray");
  q.bindValue(":idchiptray", chiptray_id);
  q.bindValue(":id_position_in_chiptray", pos_in_chiptray);
  SQLExecQuery(&q);

  q.next();
  id_chip = q.value(0).toInt();
  ROS_INFO("ID_CHIP:%d", id_chip);
  return id_chip;
}

int QNode::SQLInsertObject(int id_image,
                           int classid,
                           int roi_x1,
                           int roi_y1,
                           int roi_x2,
                           int roi_y2,
                           double score,
                           int area,
                           int point_x,
                           int point_y) {
  //QSqlQuery q("", db);
  int id_object;

  p_query->prepare("SELECT MAX(idobject) FROM 2dmms_db.object");
  SQLExecQuery(p_query);
  while (p_query->next()) {
    id_object = p_query->value(0).toInt();
    printf("id_object:%d\n", id_object);
  }
  id_object++;

  p_query->prepare(
      "INSERT INTO 2dmms_db.object (idobject, idimage_fk, class, "
      "bbox_x1, bbox_y1, bbox_x2, bbox_y2, score, area, point_x, point_y) "
      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
  p_query->addBindValue(id_object);
  p_query->addBindValue(id_image);
  p_query->addBindValue(classid);
  p_query->addBindValue(roi_x1);
  p_query->addBindValue(roi_y1);
  p_query->addBindValue(roi_x2);
  p_query->addBindValue(roi_y2);
  p_query->addBindValue(score);
  p_query->addBindValue(area);
  p_query->addBindValue(point_x);
  p_query->addBindValue(point_y);

  SQLExecQuery(p_query);
  return id_object;
}

int QNode::SQLGetMaximumSearchID(int id_chip) {
  QSqlQuery q("", db);
  int id_search;

  q.prepare("SELECT MAX(idsearch) FROM 2dmms_db.search where idchip_fk = ?");
  q.addBindValue(id_chip);
  SQLExecQuery(&q);
  q.next();
  id_search = q.value(0).toInt();

  return id_search;
}

int QNode::SQLGetMaximumEdgeID() {
  QSqlQuery q("", db);
  int id_image_edge;
  q.prepare("SELECT MAX(idimage_edge) FROM 2dmms_db.image_edge");
  SQLExecQuery(&q);
  while (q.next()) {
    id_image_edge = q.value(0).toInt();
  }
  return id_image_edge;
}

void QNode::SQLInsertSearch(int id_search, int id_chip, int step_x, int step_y) {
  //QSqlQuery q("", db);
  p_query->prepare(
      "INSERT INTO 2dmms_db.search (idsearch, idchip_fk, "
      "datetime_start, step_x, step_y) "
      "VALUES (?, ?, CURRENT_TIMESTAMP, ?, ?)");
  p_query->addBindValue(id_search);
  p_query->addBindValue(id_chip);
  p_query->addBindValue(step_x);
  p_query->addBindValue(step_y);
  p_query->exec();
  ROS_INFO("Search Record Added, id_chip:%d, id_search:%d", id_chip, id_search);
}

int QNode::SQLInsertImage(int id_search,
                          int id_chip,
                          int id_lens,
                          int pos_x,
                          int pos_y,
                          char* s3_bucket,
                          char* s3_key,
                          const char* s3_url,
                          bool send_training,
                          bool centralized,
                          bool manual) {
  //QSqlQuery q("", db);
  int id_image;

  p_query->prepare("SELECT MAX(idimage) FROM 2dmms_db.image");
  SQLExecQuery(p_query);
  while (p_query->next()) {
    id_image = p_query->value(0).toInt();
    printf("idimagealign:%d\n", id_image);
  }
  id_image++;

  p_query->prepare(
      "INSERT INTO 2dmms_db.image (idimage, "
      "idsearch_fk, idchip_fk, "
      "point_x, point_y, "
      "s3_bucket, s3_key, s3_url, idlens_fk, send_training, centralized, manual) "
      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
  p_query->addBindValue(id_image);
  p_query->addBindValue(id_search);
  p_query->addBindValue(id_chip);
  p_query->addBindValue(pos_x);
  p_query->addBindValue(pos_y);
  p_query->addBindValue(s3_bucket);
  p_query->addBindValue(s3_key);
  p_query->addBindValue(s3_url);
  p_query->addBindValue(id_lens);
  p_query->addBindValue(send_training);
  p_query->addBindValue(centralized);
  p_query->addBindValue(manual);
  SQLExecQuery(p_query);
  return id_image;
}

void QNode::SQLInsertEdgeImage(int idimage_edge,
                               int id_search,
                               int id_chip,
                               int id_lens,
                               int pos_x,
                               int pos_y,
                               char* s3_bucket,
                               char* s3_key,
                               const char* s3_url) {
  //QSqlQuery q("", db);
  p_query->prepare(
      "INSERT INTO 2dmms_db.image_edge (idimage_edge, "
      "idsearch_fk, idchip_fk, idlens_fk, point_x, point_y, "
      "s3_bucket, s3_key, s3_url) "
      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)");
  p_query->addBindValue(idimage_edge);
  p_query->addBindValue(id_search);
  p_query->addBindValue(id_chip);
  p_query->addBindValue(id_lens);
  p_query->addBindValue(pos_x);
  p_query->addBindValue(pos_y);
  p_query->addBindValue(s3_bucket);
  p_query->addBindValue(s3_key);
  p_query->addBindValue(s3_url);
  SQLExecQuery(p_query);
}

void QNode::SQLInsertAlignmentImage(int id_search,
                                    int id_chip,
                                    int id_lens,
                                    int pos_x,
                                    int pos_y,
                                    char* s3_bucket,
                                    char* s3_key,
                                    const char* s3_url) {
  int id_alignment_image;
  //QSqlQuery q("", db);

  p_query->prepare("SELECT MAX(idimagealign) FROM 2dmms_db.image_align");
  SQLExecQuery(p_query);
  while (p_query->next()) {
    id_alignment_image = p_query->value(0).toInt();
    printf("idimagealign:%d\n", id_alignment_image);
  }
  id_alignment_image++;

  p_query->prepare(
      "INSERT INTO 2dmms_db.image_align (idimagealign, "
      "idsearch_fk, idchip_fk, idlens_fk,"
      "point_x, point_y, "
      "s3_bucket, s3_key, s3_url) "
      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)");
  p_query->addBindValue(id_alignment_image);
  p_query->addBindValue(id_search);
  p_query->addBindValue(id_chip);
  p_query->addBindValue(id_lens);
  p_query->addBindValue(pos_x);
  p_query->addBindValue(pos_y);
  p_query->addBindValue(s3_bucket);
  p_query->addBindValue(s3_key);
  p_query->addBindValue(s3_url);
  SQLExecQuery(p_query);
}

QString QNode::SQLSelectMaterialName(int idchiptray) {
  QString materialname;
  QSqlQuery q("", db);
  q.prepare(
      "select material from 2dmms_db.chip where idchiptray_fk = ?");
  q.addBindValue(idchiptray);
  SQLExecQuery(&q);
  q.next();
  materialname = q.value(0).toString();
  return materialname;
}

void QNode::SetObjectiveLensTartlet(int TartletPos) {
  //////////////////////////////////////////////
  //// Set objective lens tartlet and adjust light intensity
  //////////////////////////////////////////////

  std_msgs::UInt8 rev;
  std_msgs::UInt8 inten;
  const int inten_tuple[] = {
      70,  80, 130,
      220, 250};

  rev.data = TartletPos + 1;
  lv_ncnt_n_publisher.publish(rev);
  afc_5_publisher.publish(rev);
  inten.data = inten_tuple[TartletPos];
  nic_100a_inten_publisher.publish(inten);
}

bool QNode::SQLCheckDuplicate(int id_search,
                              int id_chip,
                              int point_x,
                              int point_y,
                              int thresh) {
  /////////////////////////////////////////////////////
  //// Check if the record at current position is present in database
  ////////////////////////////////////////////////////

  QSqlQuery q("", db);
  q.prepare(
      "select count(*) from 2dmms_db.image "
      " where"
      " idsearch_fk = ?"
      " and idchip_fk = ?"
      " and centralized = true"
      " and sqrt(Power(point_x - ?, 2)"
      " + Power(point_y - ?, 2)) < ?");

  q.addBindValue(id_search);
  q.addBindValue(id_chip);
  q.addBindValue(point_x);
  q.addBindValue(point_y);
  q.addBindValue(thresh);
  SQLExecQuery(&q);
  q.next();
  int duplicate = q.value(0).toInt();
  if (duplicate == 0)
    return false;

  ROS_INFO("duplicate:%d", duplicate);
  return true;
}

bool QNode::SaveImage_affine(char* filename) {
  ////////////////////////////////////////////////////////////////////
  // Apply affine transformation to the current optical microscope image.
  // Then save transformed image in temporary folder
  ///////////////////////////////////////////////////////////////////
  HTuple hv_HomMat2DIdentity, hv_HomMat2DScale0125;
  HObject ho_GrayImage, ho_GrayImageAffinTrans;
  HObject ho_GrayImage_shift;

  HomMat2dIdentity(&hv_HomMat2DIdentity);
  HomMat2dScale(hv_HomMat2DIdentity, 0.125, 0.125, 0, 0, &hv_HomMat2DScale0125);
  Rgb1ToGray(ho_OMImage, &ho_GrayImage);
  AffineTransImage(ho_GrayImage, &ho_GrayImageAffinTrans, hv_HomMat2DScale0125,
                   "nearest_neighbor", "true");
  BitRshift(ho_GrayImageAffinTrans, &ho_GrayImage_shift, 4);
  WriteImage(ho_GrayImage_shift, "tiff", 0, filename);
  return true;
}

bool QNode::SaveImage_jpg(char* filename) {
  ///////////////////////////////////////////////////////////////////
  //// Save image in jpeg format
  ///////////////////////////////////////////////////////////////////
  try {
    HObject ho_Image_Shifted, ho_Image_Conv;
    BitRshift(ho_OMImage, &ho_Image_Shifted, 4);
    ConvertImageType(ho_Image_Shifted, &ho_Image_Conv, "byte");
    WriteImage(ho_Image_Conv, "jpeg", 0, filename);
    return true;
  } catch (HalconCpp::HException& HDevExpDefaultException) {
    printf("Error in SaveImage: %d\n", HDevExpDefaultException.ErrorCode());
    return false;
  }
}

Herror QNode::SaveImage_raw(char* filename) {
  //////////////////////////////////////////////////
  //// Save image in 12-bit raw tiff format
  //////////////////////////////////////////////////
  try {
    WriteImage(ho_OMImage, "tiff lzw", 0, filename);
    return H_MSG_OK;
  } catch (HalconCpp::HException& HDevExpDefaultException) {
    printf("Error in SaveRawImage: %d\n", HDevExpDefaultException.ErrorCode());
    return -1;
  }
}

void QNode::on_Button_StartSequence_DL_clicked() {
  on_Button_CaptureChiptray_DL_clicked();
  on_Button_DetectSiChips_DL_clicked();
  on_Button_InsertSearchRecord_DL_clicked();
  on_Button_SetKey_DL_clicked();
  on_Button_CaptureEdge_DL_clicked();
  on_Button_CaptureAlignmentImage_DL_clicked();
  on_Button_StartFind_DL_clicked();
}

void QNode::on_Button_StartSequence_RB_clicked() {
  on_Button_CaptureChiptray_RB_clicked();
  on_Button_DetectSiChips_RB_clicked();
  on_Button_InsertSearchRecord_RB_clicked();
  on_Button_SetKey_RB_clicked();
  on_Button_CaptureEdge_RB_clicked();
  on_Button_CaptureAlignmentImage_RB_clicked();
  on_Button_StartFind_RB_clicked();
}

void QNode::on_Button_PrepareManualRegistration_RB_clicked() {
  on_Button_CaptureChiptray_RB_clicked();
  on_Button_DetectSiChips_RB_clicked();
  on_Button_InsertSearchRecord_RB_clicked();
  on_Button_SetKey_RB_clicked();
  on_Button_CaptureEdge_RB_clicked();
  on_Button_CaptureAlignmentImage_RB_clicked();
}

void QNode::on_Button_PrepareManualRegistration_DL_clicked() {
  on_Button_CaptureChiptray_DL_clicked();
  on_Button_DetectSiChips_DL_clicked();
  on_Button_InsertSearchRecord_DL_clicked();
  on_Button_SetKey_DL_clicked();
  on_Button_CaptureEdge_DL_clicked();
  on_Button_CaptureAlignmentImage_DL_clicked();
}

int QNode::extractPosInChiptray(int pos_x, int pos_y) {
  int org_x = -25000;
  int org_y = 26000;
  int step = 7225;

  int row = -(pos_y - org_y)/step;
  int col = (pos_x - org_x)/step;

  ROS_INFO("row:%d, col:%d", row, col);
  int pos_in_chiptray = row*6 + col + 1;

  return pos_in_chiptray;
}

void QNode::on_Button_ManualRegistration_RB_clicked() {
  char keyname[4029];
  tdmms_finder_network_support::UploadS3 sv;
  adm2::GetCurrentPos pos_xy;

  adm2_get_current_pos_client.call(pos_xy);

  int pos_x = pos_xy.response.pos_x;
  int pos_y = pos_xy.response.pos_y;
  ROS_INFO("%d, %d", pos_x, pos_y);

  int pos_in_chiptray = extractPosInChiptray(pos_x, pos_y);
  ROS_INFO("pos_in_chiptray:%d", pos_in_chiptray);

  int row = pos_in_chiptray -1;
  int id_chip =
      SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_RB->value()),
                   tableWidget_SearchArea_RB->item(row, 10)->text().toInt());
  int id_search = SQLGetMaximumSearchID(id_chip);
  /// Grab image and stor grabbed image to ramdisk
  GrabImage(&ho_OMImage, hv_AcqHandle);
  SaveImage_jpg(temp_filename.toLocal8Bit().data());

  // upload file to S3 server
  snprintf(
      keyname, sizeof(keyname),
      "%s/%02d/man_%07d_%07d_%07d_%07d.jpg",
      lineEdit_Folder_RB->text().toLocal8Bit().data(),
      row+1,
      0,
      pos_x,
      pos_y,
      0);
  sv.request.keyname = keyname;
  finder_s3_upload_service.call(sv);
  int id_image;
  id_image = SQLInsertImage(id_search, id_chip, 0,
                            pos_x,
                            pos_y,
                            lineEdit_S3Bucket->text().toLocal8Bit().data(),
                            keyname,
                            (static_cast<std::string>(sv.response.url)).c_str(),
                            false,
                            false,
                            true);
}

void QNode::on_Button_ManualRegistration_DL_clicked() {
  char keyname[4029];
  tdmms_finder_network_support::UploadS3 sv;
  adm2::GetCurrentPos pos_xy;

  adm2_get_current_pos_client.call(pos_xy);

  int pos_x = pos_xy.response.pos_x;
  int pos_y = pos_xy.response.pos_y;
  ROS_INFO("%d, %d", pos_x, pos_y);

  int pos_in_chiptray = extractPosInChiptray(pos_x, pos_y);
  ROS_INFO("pos_in_chiptray:%d", pos_in_chiptray);

  int row = pos_in_chiptray -1;
  int id_chip =
      SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_DL->value()),
                   tableWidget_SearchArea_DL->item(row, 10)->text().toInt());
  int id_search = SQLGetMaximumSearchID(id_chip);
  /// Grab image and stor grabbed image to ramdisk
  GrabImage(&ho_OMImage, hv_AcqHandle);
  SaveImage_jpg(temp_filename.toLocal8Bit().data());

  // upload file to S3 server
  snprintf(
      keyname, sizeof(keyname),
      "%s/%02d/man_%07d_%07d_%07d_%07d.jpg",
      lineEdit_Folder_DL->text().toLocal8Bit().data(),
      row+1,
      0,
      pos_x,
      pos_y,
      0);
  sv.request.keyname = keyname;
  finder_s3_upload_service.call(sv);
  int id_image;
  id_image = SQLInsertImage(id_search, id_chip, 0,
                            pos_x,
                            pos_y,
                            lineEdit_S3Bucket->text().toLocal8Bit().data(),
                            keyname,
                            (static_cast<std::string>(sv.response.url)).c_str(),
                            false,
                            false,
                            true);
}

void QNode::on_Button_SetKey_DL_clicked() {
  /////////////////////////////////////////
  /// Generate AWS S3 parent key based on
  /// time, search id, and material name
  /////////////////////////////////////////

  if (tableWidget_SearchArea_DL->rowCount() == 0)
    return;

  QString s3ParentKey;
  QDateTime today = QDateTime::currentDateTime();
  int id_chip = SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_DL->value()),
                             tableWidget_SearchArea_DL->item(0, 10)->text().toInt());
  int id_search = SQLGetMaximumSearchID(id_chip);
  QString material = SQLSelectMaterialName(spinBox_ChiptrayID_DL->value());

  s3ParentKey = today.toString("yyyy-MM-dd");
  s3ParentKey += "_";
  s3ParentKey += QString::number(spinBox_ChiptrayID_DL->value()).rightJustified(3, '0');
  s3ParentKey += "_";
  s3ParentKey += material;
  s3ParentKey += "_run";
  s3ParentKey += QString::number(id_search).rightJustified(2,'0');
  lineEdit_Folder_DL->setText(s3ParentKey);
}

void QNode::on_Button_SetKey_RB_clicked() {
  /////////////////////////////////////////
  /// Generate AWS S3 parent key based on
  /// time, search id, and material name
  /////////////////////////////////////////

  if (tableWidget_SearchArea_RB->rowCount() == 0)
    return;

  QString s3ParentKey;
  QDateTime today = QDateTime::currentDateTime();
  int id_chip = SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_RB->value()),
                             tableWidget_SearchArea_RB->item(0, 10)->text().toInt());
  int id_search = SQLGetMaximumSearchID(id_chip);
  QString material = SQLSelectMaterialName(spinBox_ChiptrayID_RB->value());

  s3ParentKey = today.toString("yyyy-MM-dd");
  s3ParentKey += "_";
  s3ParentKey += QString::number(spinBox_ChiptrayID_RB->value()).rightJustified(3, '0');
  s3ParentKey += "_";
  s3ParentKey += material;
  s3ParentKey += "_run";
  s3ParentKey += QString::number(id_search).rightJustified(2,'0');
  lineEdit_Folder_RB->setText(s3ParentKey);
}

void QNode::on_Button_SaveImage_clicked() {
  char filename[4096];
  int piccount = spinBox_pic_count->value();
  sprintf(filename, "%s/m_%010d_%010d_%010d_%010d.jpg",
          (char*)lineEdit_picFolder->text().toStdString().c_str(),
          piccount,
          static_cast<int>(currpoint.x),
          static_cast<int>(currpoint.y),
          0000000);
  currentState = Freeze;
  ros::Duration(0.1).sleep();
  SaveImage_jpg(filename);
  currentState = Live;
  piccount++;
  spinBox_pic_count->setValue(piccount);
}

void QNode::on_Button_SaveBG_clicked() {
  currentState = Freeze;
  ros::Duration(0.1).sleep();

  GrabImage(&ho_OMImage, hv_AcqHandle);
  QString strFName = QFileDialog::getSaveFileName(
      0, tr("Save File"),
      QDir::homePath() + "/tdmms_data/background_images/untitled.tiff",
      tr("BG image file (*.tiff)"));
  if (!strFName.isEmpty()) {
    SaveImage_raw(strFName.toLocal8Bit().data());
    lineEdit_BGImage->setText(strFName);
    prepareBGImage(strFName);
  }
  currentState = Live;
}

void QNode::on_Button_LoadBG_clicked() {
  QString filename = QFileDialog::getOpenFileName(
      0, ("Open File"), QDir::homePath() + "/tdmms_data/background_images/",
      ("BG image file (*.tiff)"));
  if (!filename.isEmpty()) {
    lineEdit_BGImage->setText(filename);
    prepareBGImage(filename);
  }
}

void QNode::on_Button_StartFind_RB_clicked() {
    ////////////////////////////////////////////////////
  ///// Execute automated search using deep learning
  ////////////////////////////////////////////////////
  geometry_msgs::Point PointToSearch;
  std::list<geometry_msgs::Point> PointList;
  QTableWidgetItem* item;
  QTableWidgetItem* item_progress;
  double delay;
  tdmms_finder_network_support::UploadS3 sv;

  currentState = Search;
  for (int row = 0; row < tableWidget_SearchArea_RB->rowCount(); row++) {
    ExtractPointListFromTableWidget(tableWidget_SearchArea_RB,
                                    &PointList, row, spinBox_StepX->value(),
                                    spinBox_StepY->value());
    std_msgs::UInt8 rev;
    std_msgs::UInt8 inten;
    flakecount = 0;
    char keyname[4029];
    double progress = 0.0;
    double count = 0;
    QString str;
    int id_chip;
    int id_search;

    if (row == 0) {
      SetObjectiveLensTartlet(0);
      ros::Duration(0.5).sleep();
    }
    /////////////////////////////////////////
    // Move XY stage to the starting position
    /////////////////////////////////////////
    adm2_abs_publisher.publish(*PointList.begin());
    adm2_wait_for_stop_client.call(emp_srv);

    ///////////////////////////////////////
    /// Execute autofocus before scan starts
    ///////////////////////////////////////
    ExecuteAutofocus();
    ros::Duration(0.5).sleep();

    SetObjectiveLensTartlet(3);
    ros::Duration(5).sleep();

    ExecuteAutofocus();
    ros::Duration(1).sleep();

    ExecuteAutofocus();
    ros::Duration(1).sleep();

    item_progress = tableWidget_SearchArea_RB->item(row, 9);

    id_chip =
        SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_RB->value()),
                     tableWidget_SearchArea_RB->item(row, 10)->text().toInt());
    id_search = SQLGetMaximumSearchID(id_chip);
    // SQLSetDatetimeSearchStarted(id_search, id_chip);

    for (std::list<geometry_msgs::Point>::iterator itr = PointList.begin();
         itr != PointList.end();) {
      // Move xy stage to the next position
      adm2_abs_publisher.publish(*itr);

      /// wait until the stage motion has been finished
      delay = (static_cast<double>(spinBox_Delay->value())) / 1000;
      ros::Duration(delay).sleep();

      // count
      count++;

      // update progress bar
      progress = (count + 1) / PointList.size();
      str = QString::number(progress * 100);
      item_progress->setText(str);

      bool result;
      int pos_x, pos_y;
      unsigned int score = 0;

      /// Grab image and stor grabbed image to ramdisk
      GrabImage(&ho_OMImage, hv_AcqHandle);
      SaveImage_jpg(temp_filename.toLocal8Bit().data());

      // upload file to S3 server
      snprintf(
          keyname,
          sizeof(keyname),
          "%s/%02d/a_%07d_%07d_%07d_%07d.jpg",
          lineEdit_Folder_RB->text().toLocal8Bit().data(),
          row+1,
          flakecount,
          static_cast<int>(itr->x),
          static_cast<int>(itr->y),
          score);
      sv.request.keyname = keyname;
      finder_s3_upload_service.call(sv);
      // Run detection
      int id_image;
      id_image = SQLInsertImage(id_search,
                                id_chip,
                                0,
                                static_cast<int>(itr->x),
                                static_cast<int>(itr->y),
                                lineEdit_S3Bucket->text().toLocal8Bit().data(),
                                keyname,
                                (static_cast<std::string>(sv.response.url)).c_str(),
                                false,
                                false,
                                false);
      flakecount++;
      runRBDetection(id_image, temp_filename.toLocal8Bit().data(), itr->x, itr->y, false, false);
      ROS_INFO("ID_Image: %d", id_image);
      recordCentralizedImages_RB(id_image, 0, row, id_search, id_chip);
      itr++;
      qApp->processEvents();
    }
  }
  currentState = Live;
}

void QNode::on_Button_StartFind_DL_clicked() {
  ////////////////////////////////////////////////////
  ///// Execute automated search using deep learning
  ////////////////////////////////////////////////////
  geometry_msgs::Point PointToSearch;
  std::list<geometry_msgs::Point> PointList;
  QTableWidgetItem* item;
  QTableWidgetItem* item_progress;
  double delay;
  tdmms_finder_network_support::UploadS3 sv;

  currentState = Search;
  for (int row = 0; row < tableWidget_SearchArea_DL->rowCount(); row++) {
    ExtractPointListFromTableWidget(tableWidget_SearchArea_DL,
                                    &PointList, row, spinBox_StepX->value(),
                                    spinBox_StepY->value());
    std_msgs::UInt8 rev;
    std_msgs::UInt8 inten;
    //std_srvs::Empty emp_srv;
    //std_msgs::Empty emp;
    flakecount = 0;
    char keyname[4029];
    double progress = 0.0;
    double count = 0;
    QString str;
    int id_chip;
    int id_search;

    if (row == 0) {
      SetObjectiveLensTartlet(0);
      ros::Duration(0.5).sleep();
    }
    /////////////////////////////////////////
    // Move XY stage to the starting position
    /////////////////////////////////////////
    adm2_abs_publisher.publish(*PointList.begin());
    adm2_wait_for_stop_client.call(emp_srv);

    ///////////////////////////////////////
    /// Execute autofocus before scan starts
    ///////////////////////////////////////
    ExecuteAutofocus();
    ros::Duration(0.5).sleep();

    SetObjectiveLensTartlet(3);
    ros::Duration(5).sleep();

    ExecuteAutofocus();
    ros::Duration(1).sleep();

    ExecuteAutofocus();
    ros::Duration(1).sleep();

    item_progress = tableWidget_SearchArea_DL->item(row, 9);

    id_chip =
        SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_DL->value()),
                     tableWidget_SearchArea_DL->item(row, 10)->text().toInt());
    id_search = SQLGetMaximumSearchID(id_chip);
    // SQLSetDatetimeSearchStarted(id_search, id_chip);

    for (std::list<geometry_msgs::Point>::iterator itr = PointList.begin();
         itr != PointList.end();) {
      // Move xy stage to the next position
      adm2_abs_publisher.publish(*itr);

      // update progress bar
      progress = (count +1) / PointList.size();
      str = QString::number(progress * 100);
      item_progress->setText(str);

      /// wait until the stage motion has been finished
      delay = (static_cast<double>(spinBox_Delay->value())) / 1000;
      ros::Duration(delay).sleep();

      // count
      count++;
      // fp_tdmms_finder_Live();

      bool result;
      int pos_x, pos_y;
      unsigned int score = 0;

      /// Grab image and stor grabbed image to ramdisk
      GrabImage(&ho_OMImage, hv_AcqHandle);
      SaveImage_jpg(temp_filename.toLocal8Bit().data());

      // upload file to S3 server
      snprintf(
          keyname, sizeof(keyname),
          "%s/%02d/a_%07d_%07d_%07d_%07d.jpg",
          lineEdit_Folder_DL->text().toLocal8Bit().data(),
          row+1,
          flakecount,
          static_cast<int>(itr->x),
          static_cast<int>(itr->y),
          score);
      sv.request.keyname = keyname;
      finder_s3_upload_service.call(sv);
      // Run detection
      int id_image;
      id_image = SQLInsertImage(id_search, id_chip, 0,
                                static_cast<int>(itr->x),
                                static_cast<int>(itr->y),
                                lineEdit_S3Bucket->text().toLocal8Bit().data(),
                                keyname,
                                (static_cast<std::string>(sv.response.url)).c_str(),
                                false,
                                false,
                                false);
      flakecount++;
      runDLDetection(id_image,
                     temp_filename.toLocal8Bit().data(),
                     itr->x, itr->y,
                     false);
      recordCentralizedImages(id_image,
                              spinBox_AreaThresh_DL->value(),
                              row, id_search, id_chip);
      itr++;
      qApp->processEvents();
    }
  }
  currentState = Live;
}

int QNode::recordCentralizedImages_RB(int id_image, int area_thresh, int row, int id_search, int id_chip) {
  ROS_INFO("Capture Centralized Image");
  QSqlQuery q("", db);
  q.prepare(
      "select point_x, point_y from 2dmms_db.object where idimage_fk = ? and area > ?");
  q.addBindValue(id_image);
  q.addBindValue(area_thresh);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());

  tdmms_finder_network_support::UploadS3 sv;
  while (q.next()) {
    int pos_x = q.value(0).toInt();
    int pos_y = q.value(1).toInt();
    geometry_msgs::Point pnt;
    pnt.x = pos_x;
    pnt.y = pos_y;
    ROS_INFO("POS_X:%d, POS_Y:%d", pos_x, pos_y);

    if (SQLCheckDuplicate(id_search, id_chip, pos_x, pos_y, 20)) {
      ROS_INFO("Duplicate Image Found at (%d, %d)", pos_x, pos_y);
      continue;
    }
    adm2_abs_publisher.publish(pnt);
    ros::Duration(static_cast<double>(spinBox_Delay->value())/1000*2).sleep();
    flakecount++;

    /// Grab image and stor grabbed image to ramdisk
    GrabImage(&ho_OMImage, hv_AcqHandle);
    SaveImage_jpg(temp_filename.toLocal8Bit().data());
    char keyname[4096];
    int score = 0;
    // upload file to S3 server
    snprintf(
        keyname, sizeof(keyname),
        "%s/%02d/a_%07d_%07d_%07d_%07d.jpg",
        lineEdit_Folder_RB->text().toLocal8Bit().data(),
        row+1,
        flakecount,
        pos_x,
        pos_y,
        score);
    sv.request.keyname = keyname;
    finder_s3_upload_service.call(sv);
    int id_image;
    id_image = SQLInsertImage(id_search, id_chip,
                              0,
                              pos_x,
                              pos_y,
                              lineEdit_S3Bucket->text().toLocal8Bit().data(),
                              keyname,
                              (static_cast<std::string>(sv.response.url)).c_str(),
                              false,
                              true,
                              false);
    flakecount++;
    runRBDetection(id_image, temp_filename.toLocal8Bit().data(), pos_x, pos_y, false, false);
  }
}

int QNode::recordCentralizedImages(int id_image, int area_thresh, int row, int id_search, int id_chip) {
  ROS_INFO("Capture Centralized Image");
  QSqlQuery q("", db);
  q.prepare(
      "select point_x, point_y from 2dmms_db.object where idimage_fk = ? and area > ?");
  q.addBindValue(id_image);
  q.addBindValue(area_thresh);
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());

  tdmms_finder_network_support::UploadS3 sv;
  while(q.next()) {
    int pos_x = q.value(0).toInt();
    int pos_y = q.value(1).toInt();
    geometry_msgs::Point pnt;
    pnt.x = pos_x;
    pnt.y = pos_y;

    if (SQLCheckDuplicate(id_search, id_chip, pos_x, pos_y, 20)){
      ROS_INFO("Duplicate Image Found at (%d, %d)", pos_x, pos_y);
      continue;
    }
    adm2_abs_publisher.publish(pnt);
    ros::Duration(static_cast<double>(spinBox_Delay->value())/1000*2).sleep();
    flakecount++;

    /// Grab image and stor grabbed image to ramdisk
    GrabImage(&ho_OMImage, hv_AcqHandle);
    SaveImage_jpg(temp_filename.toLocal8Bit().data());
    char keyname[4096];
    int score = 0;
    // upload file to S3 server
    snprintf(
        keyname, sizeof(keyname),
        "%s/%02d/a_%07d_%07d_%07d_%07d.jpg",
        lineEdit_Folder_DL->text().toLocal8Bit().data(),
        row+1,
        flakecount,
        pos_x,
        pos_y,
        score);
    sv.request.keyname = keyname;
    finder_s3_upload_service.call(sv);
    int id_image;
    id_image = SQLInsertImage(id_search, id_chip, 0,
                              pos_x,
                              pos_y,
                              lineEdit_S3Bucket->text().toLocal8Bit().data(),
                              keyname,
                              (static_cast<std::string>(sv.response.url)).c_str(),
                              false,
                              true,
                              false);
    flakecount++;
    runDLDetection(id_image, temp_filename.toLocal8Bit().data(), pos_x, pos_y, false);
  }
}

void QNode::prepareBGImage(QString fname_bg) {
  ///////////////////////////////////////
  /// Prepare background image for DL 
  ///////////////////////////////////////
  ROS_INFO("%s", fname_bg.toLocal8Bit().data());
  try {
    ReadImage(&ho_Image_BG,
              HTuple(fname_bg.toLocal8Bit().data()));
  } catch(HalconCpp::HException &HDevExpDefaultException) {
    printf("Error: %d", HDevExpDefaultException.ErrorCode());
  }

  GaussFilter(ho_Image_BG, &ho_Image_BG, 11);
  GaussFilter(ho_Image_BG, &ho_Image_BG, 11);
  GaussFilter(ho_Image_BG, &ho_Image_BG, 11);
  GaussFilter(ho_Image_BG, &ho_Image_BG, 11);
  GaussFilter(ho_Image_BG, &ho_Image_BG, 11);
  GaussFilter(ho_Image_BG, &ho_Image_BG, 11);
  MeanImage(ho_Image_BG, &ho_Image_BG, 10, 10);
  MeanImage(ho_Image_BG, &ho_Image_BG, 10, 10);
  Decompose3(ho_Image_BG, &ho_Image_BG_R, &ho_Image_BG_G, &ho_Image_BG_B);
  TransFromRgb(ho_Image_BG_R, ho_Image_BG_G, ho_Image_BG_B, &ho_Image_BG_H,
               &ho_Image_BG_S, &ho_Image_BG_V, "hsv");
}

void QNode::parseRBParameters() {
  HTuple hv_null;

  hv_cent_Hs = hv_null;
  hv_delta_Hs = hv_null;
  hv_cent_Ss = hv_null;
  hv_delta_Ss = hv_null;
  hv_cent_Vs = hv_null;
  hv_delta_Vs = hv_null;
  hv_edge_lows = hv_null;
  hv_edge_highs = hv_null;
  hv_edge_alphas = hv_null;
  hv_entropy_mins = hv_null;
  hv_entropy_maxs = hv_null;
  hv_area_thresholds = hv_null;
  hv_area_holes_thresholds = hv_null;
  hv_close_edges_min_amps = hv_null;
  hv_close_edges_max_gaps = hv_null;
  hv_p_dilations = hv_null;
  hv_p_erosions = hv_null;
  hv_p_dilation2s = hv_null;
  hv_p_erosion2s = hv_null;
  hv_p_conduct_edge_detections = hv_null;
  for (int row = 0; row < tableWidget_DetectionParams_RB->rowCount(); row++) {
    hv_cent_Hs[row]    = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["cent_h"])->text().toInt();
    hv_delta_Hs[row]   = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["delta_h"])->text().toInt();
    hv_cent_Ss[row]    = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["cent_s"])->text().toInt();
    hv_delta_Ss[row]   = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["delta_s"])->text().toInt();
    hv_cent_Vs[row]    = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["cent_v"])->text().toInt();
    hv_delta_Vs[row]   = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["delta_v"])->text().toInt();
    hv_edge_lows[row]  = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["edge_low"])->text().toInt();
    hv_edge_highs[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["edge_high"])->text().toInt();
    hv_edge_alphas[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["edge_alpha"])->text().toInt();
    hv_entropy_mins[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["entropy_min"])->text().toInt();
    hv_entropy_maxs[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["entropy_max"])->text().toInt();
    hv_area_thresholds[row] =  tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["area_thresh"])->text().toInt();
    hv_area_holes_thresholds[row] =  tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["hole_thresh"])->text().toInt();
    hv_close_edges_min_amps[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["close_edge_min_amp"])->text().toInt();
    hv_close_edges_max_gaps[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["close_edges_max_gap"])->text().toInt();
    hv_p_dilations[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["dilation"])->text().toInt();
    hv_p_erosions[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["erosion"])->text().toInt();
    hv_p_conduct_edge_detections[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["edge_detect"])->text().toInt();
    hv_p_dilation2s[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["dilation2"])->text().toInt();
    hv_p_erosion2s[row] = tableWidget_DetectionParams_RB->item(row, (*map_itemColIndx)["erosion2"])->text().toInt();
  }
  //  ROS_INFO("%d", static_cast<int>(hv_cent_H.TupleLength()));
}

void QNode::runRBDetection(int id_image, char* filename,
                           int point_x, int point_y, bool test_mode, bool setting_mode) {
  parseRBParameters();
  int num_classes = static_cast<int>(hv_cent_Hs.TupleLength());
  HTuple colornames;
  colornames[0] = "orange red";
  colornames[1] = "green yellow";
  colornames[2] = "dark turquoise";

  for (int i = 0 ; i < num_classes; i++) {
    ////////////////////////////////////
    // Detection parameters
    ///////////////////////////////////
    /*
    hv_cent_H = -200;
    hv_delta_H = 600;
    hv_cent_S = -100;
    hv_delta_S = 600;
    hv_cent_V = -200;
    hv_delta_V = 100;
    hv_edge_low = 20;
    hv_edge_high = 50;
    hv_edge_alpha = 3;
    hv_entropy_min = 0;
    hv_entropy_max = 100;
    hv_area_threshold = 100;
    hv_area_holes_threshold = 1000;
    hv_close_edges_min_amp = 1;
    hv_close_edges_max_gap = 30;
    hv_p_conduct_erosion_dilation = 1;
    hv_p_dilation = 2;
    hv_p_erosion = 2;
    */
    hv_cent_H = hv_cent_Hs[i];
    hv_delta_H = hv_delta_Hs[i];
    hv_cent_S = hv_cent_Ss[i];
    hv_delta_S = hv_delta_Ss[i];
    hv_cent_V = hv_cent_Vs[i];
    hv_delta_V = hv_delta_Vs[i];
    hv_edge_low = hv_edge_lows[i];
    hv_edge_high = hv_edge_highs[i];
    hv_edge_alpha = hv_edge_alphas[i];
    hv_entropy_min = hv_entropy_mins[i];
    hv_entropy_max = hv_entropy_maxs[i];
    hv_area_threshold = hv_area_thresholds[i];
    hv_area_holes_threshold = hv_area_holes_thresholds[i];
    hv_close_edges_min_amp = hv_close_edges_min_amps[i];
    hv_close_edges_max_gap = hv_close_edges_max_gaps[i];
    hv_p_conduct_erosion_dilation = 1;
    hv_p_dilation = hv_p_dilations[i];
    hv_p_erosion = hv_p_erosions[i];
    hv_p_conduct_edge_detection = hv_p_conduct_edge_detections[i];
    hv_p_dilation2 = hv_p_dilation2s[i];
    hv_p_erosion2 = hv_p_erosion2s[i];

    GaussFilter(ho_OMImage, &ho_ImageGauss, 5);

    if(hv_edge_low > hv_edge_high)
      continue;
    // Convert colorspace of the smoothed image to HSV
    Decompose3(ho_ImageGauss, &ho_Image_R, &ho_Image_G, &ho_Image_B);
    TransFromRgb(ho_Image_R, ho_Image_G, ho_Image_B, &ho_Image_H, &ho_Image_S,
                 &ho_Image_V, "hsv");

    // Extract edges
    Rgb1ToGray(ho_OMImage, &ho_ImageGrey);
    EdgesImage(ho_ImageGrey, &ho_ImaAmp, &ho_ImaDir, "canny", hv_edge_alpha,
               "nms", hv_edge_low, hv_edge_high);
    Threshold(ho_ImaAmp, &ho_Region_Edge, 1, (HTuple(2).TuplePow(12)) - 1);

    Connection(ho_Region_Edge, &ho_connectedRegion_Edge);
    RegionFeatures(ho_connectedRegion_Edge, "area", &hv_area);
    if (0 != ((hv_area.TupleLength()) == 1)) {
      ho_RegionClosed = ho_Region_Edge;
    } else {
      CloseEdgesLength(ho_Region_Edge, ho_ImaAmp, &ho_RegionClosed,
                       hv_close_edges_min_amp, hv_close_edges_max_gap);
    }

    if (0 != hv_p_conduct_erosion_dilation) {
      if(hv_p_dilation != 0)
        DilationCircle(ho_RegionClosed, &ho_RegionClosed, hv_p_dilation);
    }
    ho_edgesRegion = ho_RegionClosed;

    // close edges which intersect window edges
    GenRegionLine(&ho_RegionLine_top, 0, 0, 0, hv_Width);
    GenRegionLine(&ho_RegionLine_bottom, hv_Height - 1, 0, hv_Height - 1,
                  hv_Width);
    GenRegionLine(&ho_RegionLine_left, 0, 0, hv_Height, 0);
    GenRegionLine(&ho_RegionLine_right, 0, hv_Width - 1, hv_Height,
                  hv_Width - 1);

    if (0 != hv_p_conduct_erosion_dilation) {
      if(hv_p_dilation != 0){
        DilationCircle(ho_RegionLine_top, &ho_RegionLine_top, hv_p_dilation);
        DilationCircle(ho_RegionLine_bottom, &ho_RegionLine_bottom,
                       hv_p_dilation);
        DilationCircle(ho_RegionLine_left, &ho_RegionLine_left, hv_p_dilation);
        DilationCircle(ho_RegionLine_right, &ho_RegionLine_right, hv_p_dilation);
      }
    }

    Union2(ho_RegionClosed, ho_RegionLine_top, &ho_RegionClosed);
    FillUp(ho_RegionClosed, &ho_RegionClosed);
    Difference(ho_RegionClosed, ho_RegionLine_top, &ho_RegionClosed);

    Union2(ho_RegionClosed, ho_RegionLine_bottom, &ho_RegionClosed);
    FillUp(ho_RegionClosed, &ho_RegionClosed);
    Difference(ho_RegionClosed, ho_RegionLine_bottom, &ho_RegionClosed);

    Union2(ho_RegionClosed, ho_RegionLine_left, &ho_RegionClosed);
    FillUp(ho_RegionClosed, &ho_RegionClosed);
    Difference(ho_RegionClosed, ho_RegionLine_left, &ho_RegionClosed);

    Union2(ho_RegionClosed, ho_RegionLine_right, &ho_RegionClosed);
    FillUp(ho_RegionClosed, &ho_RegionClosed);
    Difference(ho_RegionClosed, ho_RegionLine_right, &ho_RegionClosed);

    Difference(ho_RegionClosed, ho_edgesRegion, &ho_RegionDifference);

    if (0 != hv_p_conduct_erosion_dilation) {
      if( 0!= hv_p_erosion)
        ErosionCircle(ho_RegionDifference, &ho_RegionDifference, hv_p_erosion);
    }

    BitRshift(ho_ImageGrey, &ho_ImageGrey_conv, 4);
    ConvertImageType(ho_ImageGrey_conv, &ho_ImageGrey_8, "byte");
    SelectGray(ho_RegionDifference, ho_ImageGrey_8, &ho_RegionS, "entropy",
               "and", hv_entropy_min, hv_entropy_max);

    // Threshold Image
    SubImage(ho_Image_H, ho_Image_BG_H, &ho_ImageSub_H, 1, 4096 / 2);
    SubImage(ho_Image_S, ho_Image_BG_S, &ho_ImageSub_S, 1, 4096 / 2);
    SubImage(ho_Image_V, ho_Image_BG_V, &ho_ImageSub_V, 1, 4096 / 2);

    hv_lb_H = (hv_cent_H-hv_delta_H)+(4096/2);
    TupleMax2((hv_cent_H+hv_delta_H)+(4096/2), 0, &hv_ub_H);

    hv_lb_S = (hv_cent_S-hv_delta_S)+(4096/2);
    TupleMax2((hv_cent_S+hv_delta_S)+(4096/2), 0, &hv_ub_S);

    hv_lb_V = (hv_cent_V-hv_delta_V)+(4096/2);
    TupleMax2((hv_cent_V+hv_delta_V)+(4096/2), 0, &hv_ub_V);

    //threshold (ImageSub_H, Region_H, cent_H - delta_H + 4096/2, cent_H + delta_H + 4096/2)
    Threshold(ho_ImageSub_H, &ho_Region_H, hv_lb_H, hv_ub_H);
    Threshold(ho_ImageSub_S, &ho_Region_S, hv_lb_S, hv_ub_S);
    Threshold(ho_ImageSub_V, &ho_Region_V, hv_lb_V, hv_ub_V);

    Intersection(ho_Region_H, ho_Region_S, &ho_RegionI);
    Intersection(ho_Region_V, ho_RegionI, &ho_RegionI);

    if (0 != hv_p_conduct_edge_detection) {
      Intersection(ho_RegionI, ho_RegionS, &ho_RegionIII);
    } else {
      ho_RegionIII = ho_RegionI;
    }

    //Intersection(ho_RegionI, ho_RegionS, &ho_RegionIII);
    Connection(ho_RegionIII, &ho_RegionIII);
    ClosingCircle(ho_RegionIII, &ho_RegionIII, 3);
    ErosionCircle(ho_RegionIII, &ho_RegionIII, hv_p_erosion2);
    DilationCircle(ho_RegionIII, &ho_RegionIII, hv_p_dilation2);

    // select_shape (RegionIII, SelectedRegions, ['area', 'area_holes'], 'and',
    // [area_threshold, 0], ['max', area_holes_threshold])
    SelectShape(ho_RegionIII, &ho_SelectedRegions, "area", "and",
                hv_area_threshold, "max");
    AreaCenter(ho_SelectedRegions, &hv_Areas, &hv_PointsY, &hv_PointsX);
    CountObj(ho_SelectedRegions, &hv_Number);
    /*if (0 != (hv_Number > 0)) {
      TupleMax(hv_Areas, &hv_maxvalue);
      TupleFind(hv_Areas, hv_maxvalue, &hv_index);
      SelectObj(ho_SelectedRegions, &ho_SelectedFlake, hv_index + 1);
      hv_area = ((const HTuple &)hv_feature_values)[0];
      }*/

    // Display OM Image
    HDevWindowStack::SetActive(hv_WindowHandleOM);
    if (HDevWindowStack::IsOpen())
      if (i==0)
        DispObj(ho_OMImage, HDevWindowStack::GetActive());
    {
      std::vector<int> db_object_id;
      HTuple end_val181 = hv_Number;
      HTuple step_val181 = 1;
      for (hv_index = 1; hv_index.Continue(end_val181, step_val181);
           hv_index += step_val181) {
        SelectObj(ho_SelectedRegions, &ho_SelectedFlake, hv_index);
        GetRegionPolygon(ho_SelectedFlake, 2, &hv_Rows, &hv_Columns);

        int lineWidth = 4;
        SetLineWidth(hv_WindowHandleOM, lineWidth);
        SetColor(hv_WindowHandleOM, colornames[i%3]);
        DispPolygon(hv_WindowHandleOM, hv_Rows, hv_Columns);

        //////////////////
        /// Extract ROI
        /////////////////
        HTuple roi_x1, roi_x2, roi_y1, roi_y2;
        TupleMin(hv_Columns, &roi_x1);
        TupleMax(hv_Columns, &roi_x2);
        TupleMin(hv_Rows, &roi_y1);
        TupleMax(hv_Rows, &roi_y2);

        //////////////////
        /// Draw ROI
        /////////////////
        DispLine(hv_WindowHandleOM,
                 roi_y1 - lineWidth*2,
                 roi_x1 - lineWidth*2,
                 roi_y1 - lineWidth*2,
                 roi_x2 + lineWidth*2);
        DispLine(hv_WindowHandleOM,
                 roi_y1 - lineWidth*2,
                 roi_x2 + lineWidth*2,
                 roi_y2 + lineWidth*2,
                 roi_x2 + lineWidth*2);
        DispLine(hv_WindowHandleOM,
                 roi_y2 + lineWidth*2,
                 roi_x2 + lineWidth*2,
                 roi_y2 + lineWidth*2,
                 roi_x1 - lineWidth*2);
        DispLine(hv_WindowHandleOM,
                 roi_y2 + lineWidth*2,
                 roi_x1 - lineWidth*2,
                 roi_y1 - lineWidth*2,
                 roi_x1 - lineWidth*2);
        //////////////////
        /// Extract area region
        /////////////////
        HTuple hv_Area, hv_PointY, hv_PointX;
        AreaCenter(ho_SelectedFlake, &hv_Area, &hv_PointY, &hv_PointX);
        if(!test_mode) {
          int class_id = tableWidget_DetectionParams_RB->item(i, (*map_itemColIndx)["class"])->text().toInt();
          int pos_x = static_cast<int>(point_x
                                       + 120.0/2000.0
                                       * ((static_cast<int>(roi_x2)
                                           + static_cast<int>(roi_x1))/2
                                          - 1000));
          int pos_y = static_cast<int>(point_y
                                       - 120.0/2000.0
                                       * ((static_cast<int>(roi_y2)
                                           + static_cast<int>(roi_y1))/2
                                          - 500));
          int id_db_object =
              SQLInsertObject(id_image, class_id, static_cast<int>(roi_x1), static_cast<int>(roi_y1), 
                              static_cast<int>(roi_x2), static_cast<int>(roi_y2), 1.0, static_cast<int>(hv_Area),
                              pos_x, pos_y);
          ROS_INFO("Det_POS_X:%d, POS_Y:%d", pos_x,pos_y);
          int mask_id = 0;
          QVariantList idmasks, idpoints, idobject_fks, point_xs, point_ys;
          for (int k=0 ; k < hv_Rows.TupleLength(); k++) {
            idmasks << mask_id;
            idpoints << k;
            idobject_fks << id_db_object;
            point_xs << static_cast<int>(hv_Columns[k]);
            point_ys << static_cast<int>(hv_Rows[k]);
          }
          QSqlQuery q("", db);
          q.prepare(
              "INSERT INTO 2dmms_db.mask (idmask, idpoint, idobject_fk, "
              "point_x, point_y) "
              "VALUES (?, ?, ?, ?, ?)");
          q.addBindValue(idmasks);
          q.addBindValue(idpoints);
          q.addBindValue(idobject_fks);
          q.addBindValue(point_xs);
          q.addBindValue(point_ys);
          if (!q.execBatch())
            ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                      q.lastError().text().toStdString().c_str(),
                      q.lastQuery().toStdString().c_str());
          else
            ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
          //db_object_id.push_back(id_db_object);
        }
      }
      if(i==1)
        ros::Duration(1).sleep();
    }

    if (setting_mode) {
      if (i == tableWidget_DetectionParams_RB->currentRow()) {
        try {
          HDevWindowStack::SetActive(hv_WindowHandle_H);
          if (HDevWindowStack::IsOpen())
            DispObj(ho_Image_H, HDevWindowStack::GetActive());
          HDevWindowStack::SetActive(hv_WindowHandle_S);
          if (HDevWindowStack::IsOpen())
            DispObj(ho_Image_S, HDevWindowStack::GetActive());
          HDevWindowStack::SetActive(hv_WindowHandle_V);
          if (HDevWindowStack::IsOpen())
            DispObj(ho_Image_V, HDevWindowStack::GetActive());

          //* Display Edges
          HDevWindowStack::SetActive(hv_WindowHandle_Edge);
          if (HDevWindowStack::IsOpen()) {
            ClearWindow(HDevWindowStack::GetActive());
            DispObj(ho_edgesRegion, HDevWindowStack::GetActive());
          }

          //* Display region enclosed by edges
          HDevWindowStack::SetActive(hv_WindowHandle_ClosedRegions);
          if (HDevWindowStack::IsOpen()) {
            ClearWindow(HDevWindowStack::GetActive());
            DispObj(ho_RegionDifference, HDevWindowStack::GetActive());
          }

          HDevWindowStack::SetActive(hv_WindowHandle_H_th);
          if (HDevWindowStack::IsOpen()) {
            ClearWindow(HDevWindowStack::GetActive());
            DispObj(ho_Region_H, HDevWindowStack::GetActive());
          }

          HDevWindowStack::SetActive(hv_WindowHandle_S_th);
          if (HDevWindowStack::IsOpen()) {
            ClearWindow(HDevWindowStack::GetActive());
            DispObj(ho_Region_S, HDevWindowStack::GetActive());
          }

          HDevWindowStack::SetActive(hv_WindowHandle_V_th);
          if (HDevWindowStack::IsOpen()) {
            ClearWindow(HDevWindowStack::GetActive());
            DispObj(ho_Region_V, HDevWindowStack::GetActive());
          }

          HDevWindowStack::SetActive(hv_WindowHandle_Result);
          if (HDevWindowStack::IsOpen()) {
            ClearWindow(HDevWindowStack::GetActive());
            DispObj(ho_SelectedRegions, HDevWindowStack::GetActive());
          }
        } catch(HalconCpp::HException &HDevExpDefaultException) {
          printf("Error: %d", HDevExpDefaultException.ErrorCode());
        }
      }
    }
  }
}

void QNode::runDLDetection(int id_image, char* filename, int point_x, int point_y, bool test_mode) {
  char commands[4096];
  //////////////////////////
  // Run Inference
  /////////////////////////
  snprintf(
      commands,
      sizeof(commands),
      "python3 "
      "%s/tdmms_ws/src/tdmms_finder/tdmms_finder_dl/dl_infer.py "
      "%s",
      QDir::homePath().toStdString().c_str(),
      filename);
  printf("%s",commands);
  system(commands);

  ///////////////////////////////////////
  /// Read results of inference -- scores
  ///////////////////////////////////////
  std::ifstream f;
  f.open("/ramdisk/tmp_scores.csv");
  if (!f.is_open()) {
    std::cerr << "error: file open failed.\n";
    return;
  }
  std::string line_score;
  std::getline(f, line_score);
  std::vector<std::string> strvec_score = split(line_score, ',');

  std::vector<double> scores;
  for (int i = 0; i < strvec_score.size(); i++) {
    scores.push_back(std::atof(strvec_score.at(i).c_str()));
  }
  f.close();

  ///////////////////////////////////////
  /// Read results of inference -- rois
  ///////////////////////////////////////
  f.open("/ramdisk/tmp_roi.csv");
  if (!f.is_open()) {
    std::cerr << "error: file open failed.\n";
    return;
  }
  std::string line, val;
  std::vector<int> array;

  while (std::getline(f, line)) {
    //std::vector<int> v;
    std::stringstream s(line);
    while (getline(s, val, ',')) {
      std::istringstream iss1(val);
      int val_int;
      iss1 >> val_int;
      array.push_back(val_int); /* add to row vector */
    }
  }
  f.close();

  ///////////////////////////////////////
  //// Parse values into arrays
  ///////////////////////////////////////
  std::vector<int> roi_y1, roi_x1, roi_y2, roi_x2;
  std::vector<int> db_object_id;
  for (std::vector<int>::const_iterator it = array.begin(), e = array.end();
       it != e; ++it) {
    roi_y1.push_back(*it);
    roi_x1.push_back(*(++it));
    roi_y2.push_back(*(++it));
    roi_x2.push_back(*(++it));
  }

  ///////////////////////////////////////
  /// Read results of inference -- masks
  ///////////////////////////////////////
  f.open("/ramdisk/tmp_masks.csv");
  if (!f.is_open()) {
    std::cerr << "error: file open failed.\n";
    return;
  }
  std::string line2, val2; /* string for line & value */
  std::vector<int> array2; /* vector of vector<int>  */

  std::vector<int> array_obj_id;
  std::vector<int> array_mask_id;
  std::vector<int> array_mask_row;
  std::vector<int> array_mask_class;
  std::vector<int> array_mask_column;

  while (std::getline(f, line2)) {/* read each line */
    std::vector<std::string> strvec = split(line2, ',');
    array_obj_id.push_back(std::atoi(strvec.at(0).c_str()));
    array_mask_id.push_back(std::atoi(strvec.at(1).c_str()));
    array_mask_class.push_back(std::atoi(strvec.at(2).c_str()));
    array_mask_column.push_back(std::atoi(strvec.at(3).c_str()));
    array_mask_row.push_back(std::atoi(strvec.at(4).c_str()));
  }
  f.close();

  ///////////////////////////////////////////
  /// Read results of inference -- class ids
  ///////////////////////////////////////////
  f.open("/ramdisk/tmp_class.csv");
  if (!f.is_open()) {
    std::cerr << "error: file open failed.\n";
    return;
  }
  std::string line3, val3;
  std::vector<int> array_class;
  while (std::getline(f, line3)) {
    std::vector<std::string> strvec = split(line3, ',');
    for (int i=0; i < strvec.size(); i++) {
      array_class.push_back(std::atoi(strvec.at(i).c_str()));
    }
  }
  f.close();

  //////////////////////////////////////
  /// Calculate areas of masks
  //////////////////////////////////////
  std::vector<int> array_mask_areas;
  int areas[4096];
  for (int i = 0; i < 4096; i++)
    areas[i] = 0;

  for (int i = 0; i < array_obj_id.size();) {
    int obj_id = array_obj_id.at(i);
    int mask_id = array_mask_id.at(i);
    HTuple rows;
    HTuple columns;

    int j = 0;
    do {
      rows[j] = array_mask_row.at(i);
      columns[j] = array_mask_column.at(i);
      j++;
      i++;
      if (i == array_obj_id.size()) break;
    } while (obj_id == array_obj_id.at(i) && mask_id == array_mask_id.at(i));
    HObject ho_Pol, ho_Filled;
    GenRegionPolygon(&ho_Pol, rows, columns);
    FillUp(ho_Pol, &ho_Filled);
    HTuple ho_RegionArea, ho_RegionRow, ho_RegionColumn;
    AreaCenter(ho_Filled, &ho_RegionArea, &ho_RegionRow, &ho_RegionColumn);
    int area = static_cast<int>(ho_RegionArea);
    areas[obj_id] += area;
  }

  //////////////////////////////////////////
  /// Insert object record to SQL Sever
  //////////////////////////////////////////
  if (!test_mode) {
    for (int i = 0; i < roi_x1.size(); i++) {
      int pos_x = static_cast<int>(point_x + 120.0/2000.0 * ((roi_x2.at(i) + roi_x1.at(i))/2 - 1000));
      int pos_y = static_cast<int>(point_y - 120.0/2000.0 * ((roi_y2.at(i) + roi_y2.at(i))/2 - 500));
      int id_db_object =
          SQLInsertObject(id_image, array_class.at(i), roi_x1.at(i), roi_y1.at(i),
                          roi_x2.at(i), roi_y2.at(i), scores.at(i), areas[i],
                          pos_x, pos_y);
      db_object_id.push_back(id_db_object);
    }
  }
  ///////////////////////////////////////
  /// Display results of inference
  ///////////////////////////////////////
  DispObj(ho_OMImage, hv_WindowHandleOM);

  ///////////////////////////////////////
  //// Insert Mask Items
  //////////////////////////////////////

  QVariantList idmasks, idpoints, idobject_fks, point_xs, point_ys;

  for (int i = 0; i < array_obj_id.size();) {
    int obj_id = array_obj_id.at(i);
    int mask_id = array_mask_id.at(i);
    HTuple rows;
    HTuple columns;

    int j = 0;
    do {
      rows[j] = array_mask_row.at(i);
      columns[j] = array_mask_column.at(i);
      j++;
      i++;
      if (i == array_obj_id.size()) break;
    } while (obj_id == array_obj_id.at(i) && mask_id == array_mask_id.at(i));
    int num_corners = j;
    std::string label;
    switch (array_mask_class.at(i - 1)) {
      case 1:
        SetColor(hv_WindowHandleOM, "red");
        label = "mono";
        break;
      case 2:
        SetColor(hv_WindowHandleOM, "green");
        label = "few";
        break;
      case 3:
        SetColor(hv_WindowHandleOM, "blue");
        label = "thick";
        break;
      default:
        SetColor(hv_WindowHandleOM, "black");
        std::stringstream temp;
        temp << array_mask_class.at(i-1);
        label = temp.str();
        break;
    }
    int lineWidth = 4;
    SetLineWidth(hv_WindowHandleOM, lineWidth);

    DispPolygon(hv_WindowHandleOM, rows, columns);

    DispLine(hv_WindowHandleOM,
             roi_y1.at(obj_id) - lineWidth, roi_x1.at(obj_id) - lineWidth,
             roi_y1.at(obj_id) - lineWidth, roi_x2.at(obj_id) + lineWidth);
    DispLine(hv_WindowHandleOM,
             roi_y1.at(obj_id) - lineWidth, roi_x2.at(obj_id) + lineWidth,
             roi_y2.at(obj_id) + lineWidth, roi_x2.at(obj_id) + lineWidth);
    DispLine(hv_WindowHandleOM,
             roi_y2.at(obj_id) + lineWidth, roi_x2.at(obj_id) + lineWidth,
             roi_y2.at(obj_id) + lineWidth, roi_x1.at(obj_id) - lineWidth);
    DispLine(hv_WindowHandleOM,
             roi_y2.at(obj_id) + lineWidth, roi_x1.at(obj_id) - lineWidth,
             roi_y1.at(obj_id) - lineWidth, roi_x1.at(obj_id) - lineWidth);
    halcon_bridge::disp_message(hv_WindowHandleOM, label.c_str(), "window",
                                roi_y1.at(obj_id) - 20, roi_x1.at(obj_id),
                                "white", "false");
    std::ostringstream oss;
    oss << scores.at(obj_id);
    halcon_bridge::disp_message(hv_WindowHandleOM, oss.str().c_str(), "window",
                                roi_y1.at(obj_id) - 35, roi_x1.at(obj_id),
                                "white", "false");
    if (!test_mode) {
      for (int k=0; k < num_corners; k++) {
        idmasks << mask_id;
        idpoints << k;
        idobject_fks << db_object_id.at(obj_id);
        point_xs << static_cast<int>(columns[k]);
        point_ys << static_cast<int>(rows[k]);
      }
    }
  }

  if (!test_mode) {
    if (checkBox_RecordMask_DL->isChecked()) {
      QSqlQuery q("", db);
      q.prepare(
          "INSERT INTO 2dmms_db.mask (idmask, idpoint, idobject_fk, "
          "point_x, point_y) "
          "VALUES (?, ?, ?, ?, ?)");
      q.addBindValue(idmasks);
      q.addBindValue(idpoints);
      q.addBindValue(idobject_fks);
      q.addBindValue(point_xs);
      q.addBindValue(point_ys);

      if (!q.execBatch())
        ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                  q.lastError().text().toStdString().c_str(),
                  q.lastQuery().toStdString().c_str());
      else
        ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
    }
  }
}

void QNode::CaptureChiptray(int id_chiptray) {
  int step_x_original = step_x;
  double delay = 0.3;
  char filename[4092];
  int i, j;
  geometry_msgs::Point movpoint;
  std_msgs::UInt8 rev;
  std_msgs::UInt8 inten;
  //  std_srvs::Empty emp_srv;

  /////////////////////////////////////////
  /// ROS Information
  /////////////////////////////////////////
  ROS_INFO("CaptureChiptray Started");

  ///////////////////////////////////////
  // Set objective lens to 5x 
  //////////////////////////////////////
  SetObjectiveLensTartlet(0);
  ROS_INFO("Set objective lens tartlet");
  ros::Duration(1).sleep();

  ///////////////////////////////////////
  // Set XY Stage Speed
  //////////////////////////////////////
  adm2::velocity vel;
  vel.velocity_low = 500;
  vel.velocity_high = 10000;
  vel.accl = 100;
  adm2_vel_publisher.publish(vel);

  ///////////////////////////////////////
  // Move XY Stage and wait until XY stage
  // motion has been finished
  //////////////////////////////////////
  ros::Duration(0.1).sleep();
  adm2_abs_publisher.publish(origin);
  adm2_wait_for_stop_client.call(emp_srv);

  currentState = Search;

  //////////////////////////////////////
  // Main loop
  //////////////////////////////////////
  int numofchips = SQLGetNumOfChips(id_chiptray);
  int numsteps_y_end = numsteps_y/6 * ((numofchips-1)/6+1);
  ROS_INFO("Numsteps_y_end: %d", numsteps_y_end);
  for (i = 0; i < numsteps_y_end; i++) {
    for (j = 0; j < numsteps_x; j++) {
      snprintf(filename, sizeof(filename), "%s/%d.tif",
               strDefaultTemporaryFolder.toLocal8Bit().data(),
               j + i * numsteps_x);
      ROS_INFO("%s", filename);
      ROS_INFO("i:%d", i);
      GrabImage(&ho_OMImage, hv_AcqHandle);
      DispObj(ho_OMImage, hv_WindowHandleOM);
      SaveImage_affine(filename);
      /////////////////////////////////
      // Move XY Stage to the next point
      /////////////////////////////////
      movpoint.x = step_x;
      movpoint.y = 0;
      adm2_stp_publisher.publish(movpoint);
      ros::Duration(delay).sleep();

      //////////////////////////////////
      // Process QT Events
      //////////////////////////////////
      qApp->processEvents();
    }
    ros::Duration(delay).sleep();

    ///////////////////////////////////////
    // Move XY Stage to the next point
    ///////////////////////////////////////
    movpoint.x = 0;
    movpoint.y = step_y;
    adm2_stp_publisher.publish(movpoint);
    ros::Duration(delay).sleep();
    step_x = -step_x;
  }
  ///////////////////////////////////////
  // Move XY Stage to origin and wait until XY stage
  // motion has been finished
  //////////////////////////////////////
  ros::Duration(0.1).sleep();
  adm2_abs_publisher.publish(origin);
  adm2_wait_for_stop_client.call(emp_srv);
  GrabImage(&ho_OMImage, hv_AcqHandle);
  DispObj(ho_OMImage, hv_WindowHandleOM);

  for (i = numsteps_y_end; i < numsteps_y; i++) {
    for (j = 0; j < numsteps_x; j++) {
      snprintf(filename, sizeof(filename), "%s/%d.tif",
               strDefaultTemporaryFolder.toLocal8Bit().data(),
               j + i * numsteps_x);
      ROS_INFO("%s", filename);
      SaveImage_affine(filename);
      qApp->processEvents();
    }
  }
  step_x = step_x_original;
  currentState = Live;
}

void QNode::on_Button_CaptureChiptray_DL_clicked() {
  int id_chiptray =  static_cast<int>(spinBox_ChiptrayID_DL->value());
  CaptureChiptray(id_chiptray);
}

void QNode::on_Button_CaptureChiptray_RB_clicked() {
  int id_chiptray =  static_cast<int>(spinBox_ChiptrayID_RB->value());
  CaptureChiptray(id_chiptray);
}

void QNode::on_Button_DetectSiChips_RB_clicked() {
  int i, j;
  int numchip;
  int pos_x1[256], pos_x2[256], pos_x3[256], pos_x4[256];
  int pos_y1[256], pos_y2[256], pos_y3[256], pos_y4[256];
  int pos_in_chiptray[36];
  int chipid[36];

  double scaling_factor = (static_cast<double>(step_x)) / 2034.0 * 8.0;
  double dbuf;

  // Local iconic variables
  HObject ho_GrayImage_list, ho_Image, ho_TiledImage, ho_Region1;
  HObject ho_ConnectedRegions, ho_Circle1, ho_Circle2;
  HObject ho_Circle3, ho_Circle4, ho_Circles;
  HObject ho_ConnectedRegions_sort;

  // Local control variables
  HTuple hv_exposure;
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
  for (i = 0; i < numsteps_x * numsteps_y; i++) hv_Tuple11[i] = -1;

  for (i = 0; i < numsteps_y; i++) {
    for (j = 0; j < numsteps_x; j++) {
      hv_Tuple_row[j + (i * numsteps_x)] = (i * 1016) / 8;
      if ((i % 2) == 0) {
        hv_Tuple_col[j + (i * numsteps_x)] = (j * 2034) / 8;
      } else {
        hv_Tuple_col[j + (i * numsteps_x)] = ((numsteps_x - j) * 2034) / 8;
      }
    }
  }

  GenEmptyObj(&ho_GrayImage_list);
  for (i = numsteps_x * numsteps_y - 1; i >= 0; i--) {
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
  numchip = static_cast<int>(hv_count);

  for (i = 0; i < numchip; i++) {
    dbuf = hv_ColPoint1[i];
    pos_x1[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_ColPoint2[i];
    pos_x2[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_ColPoint3[i];
    pos_x4[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_ColPoint4[i];
    pos_x3[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;

    dbuf = hv_RowPoint1[i];
    pos_y1[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;
    dbuf = hv_RowPoint2[i];
    pos_y2[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;
    dbuf = hv_RowPoint3[i];
    pos_y4[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;
    dbuf = hv_RowPoint4[i];
    pos_y3[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;

    int pos_center_x =
        static_cast<int>((pos_x1[i] + pos_x2[i] + pos_x3[i] + pos_x4[i]) / 4);
    int pos_center_y =
        static_cast<int>((pos_y1[i] + pos_y2[i] + pos_y3[i] + pos_y4[i]) / 4);
    pos_in_chiptray[i] = static_cast<int>((pos_center_x - (-23966)) / 7000) -
                         6 * static_cast<int>((pos_center_y - (24000)) / 7000) +
                         1;

    // Get Chip ID
    chipid[i] = SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_RB->value()),
                             pos_in_chiptray[i]);
  }

  for (i = 0; i < numchip; i++) {
    on_Button_Add_RB_clicked();
    tableWidget_SearchArea_RB->item(i, 0)->setText(QString::number(pos_x1[i]));
    tableWidget_SearchArea_RB->item(i, 1)->setText(QString::number(pos_y1[i]));
    tableWidget_SearchArea_RB->item(i, 2)->setText(QString::number(pos_x2[i]));
    tableWidget_SearchArea_RB->item(i, 3)->setText(QString::number(pos_y2[i]));
    tableWidget_SearchArea_RB->item(i, 4)->setText(QString::number(pos_x3[i]));
    tableWidget_SearchArea_RB->item(i, 5)->setText(QString::number(pos_y3[i]));
    tableWidget_SearchArea_RB->item(i, 6)->setText(QString::number(pos_x4[i]));
    tableWidget_SearchArea_RB->item(i, 7)->setText(QString::number(pos_y4[i]));
    tableWidget_SearchArea_RB->item(i, 10)
        ->setText(QString::number(pos_in_chiptray[i]));
    tableWidget_SearchArea_RB->item(i, 11)->setText(QString::number(chipid[i]));
  }
}

void QNode::on_Button_DetectSiChips_DL_clicked() {
  int i, j;
  int numchip;
  int pos_x1[256], pos_x2[256], pos_x3[256], pos_x4[256];
  int pos_y1[256], pos_y2[256], pos_y3[256], pos_y4[256];
  int pos_in_chiptray[36];
  int chipid[36];

  double scaling_factor = (static_cast<double>(step_x)) / 2034.0 * 8.0;
  double dbuf;

  // Local iconic variables
  HObject ho_GrayImage_list, ho_Image, ho_TiledImage, ho_Region1;
  HObject ho_ConnectedRegions, ho_Circle1, ho_Circle2;
  HObject ho_Circle3, ho_Circle4, ho_Circles;
  HObject ho_ConnectedRegions_sort;

  // Local control variables
  HTuple hv_exposure;
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
  for (i = 0; i < numsteps_x * numsteps_y; i++) hv_Tuple11[i] = -1;

  for (i = 0; i < numsteps_y; i++) {
    for (j = 0; j < numsteps_x; j++) {
      hv_Tuple_row[j + (i * numsteps_x)] = (i * 1016) / 8;
      if ((i % 2) == 0) {
        hv_Tuple_col[j + (i * numsteps_x)] = (j * 2034) / 8;
      } else {
        hv_Tuple_col[j + (i * numsteps_x)] = ((numsteps_x - j) * 2034) / 8;
      }
    }
  }

  GenEmptyObj(&ho_GrayImage_list);
  for (i = numsteps_x * numsteps_y - 1; i >= 0; i--) {
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
  numchip = static_cast<int>(hv_count);

  for (i = 0; i < numchip; i++) {
    dbuf = hv_ColPoint1[i];
    pos_x1[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_ColPoint2[i];
    pos_x2[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_ColPoint3[i];
    pos_x4[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_ColPoint4[i];
    pos_x3[i] = static_cast<int>(dbuf * scaling_factor) -
                static_cast<int>(step_x / 2) + origin.x;

    dbuf = hv_RowPoint1[i];
    pos_y1[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;
    dbuf = hv_RowPoint2[i];
    pos_y2[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;
    dbuf = hv_RowPoint3[i];
    pos_y4[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;
    dbuf = hv_RowPoint4[i];
    pos_y3[i] = -static_cast<int>(dbuf * scaling_factor) +
                static_cast<int>(step_y / 2) + origin.y;

    int pos_center_x =
        static_cast<int>((pos_x1[i] + pos_x2[i] + pos_x3[i] + pos_x4[i]) / 4);
    int pos_center_y =
        static_cast<int>((pos_y1[i] + pos_y2[i] + pos_y3[i] + pos_y4[i]) / 4);
    pos_in_chiptray[i] = static_cast<int>((pos_center_x - (-23966)) / 7000) -
                         6 * static_cast<int>((pos_center_y - (24000)) / 7000) +
                         1;

    // Get Chip ID
    chipid[i] = SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_DL->value()),
                             pos_in_chiptray[i]);
  }

  for (i = 0; i < numchip; i++) {
    on_Button_Add_DL_clicked();
    tableWidget_SearchArea_DL->item(i, 0)
        ->setText(QString::number(pos_x1[i]));
    tableWidget_SearchArea_DL->item(i, 1)
        ->setText(QString::number(pos_y1[i]));
    tableWidget_SearchArea_DL->item(i, 2)
        ->setText(QString::number(pos_x2[i]));
    tableWidget_SearchArea_DL->item(i, 3)
        ->setText(QString::number(pos_y2[i]));
    tableWidget_SearchArea_DL->item(i, 4)
        ->setText(QString::number(pos_x3[i]));
    tableWidget_SearchArea_DL->item(i, 5)
        ->setText(QString::number(pos_y3[i]));
    tableWidget_SearchArea_DL->item(i, 6)
        ->setText(QString::number(pos_x4[i]));
    tableWidget_SearchArea_DL->item(i, 7)
        ->setText(QString::number(pos_y4[i]));
    tableWidget_SearchArea_DL->item(i, 10)
        ->setText(QString::number(pos_in_chiptray[i]));
    tableWidget_SearchArea_DL->item(i, 11)
        ->setText(QString::number(chipid[i]));
  }
}

void QNode::on_Button_InsertSearchRecord_DL_clicked() {
  int numofchips =
      SQLGetNumOfChips(static_cast<int>(spinBox_ChiptrayID_DL->value()));
  for (int i = 0; i < numofchips; i++) {
    int id_chip =
        SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_DL->value()),
                     tableWidget_SearchArea_DL->item(i, 10)->text().toInt());
    int id_search = SQLGetMaximumSearchID(id_chip) + 1;
    SQLInsertSearch(id_search, id_chip,
                    spinBox_StepX->value(), spinBox_StepY->value());
  }
}

void QNode::on_Button_InsertSearchRecord_RB_clicked() {
  int numofchips =
      SQLGetNumOfChips(static_cast<int>(spinBox_ChiptrayID_RB->value()));
  for (int i = 0; i < numofchips; i++) {
    int id_chip =
        SQLGetChipID(static_cast<int>(spinBox_ChiptrayID_RB->value()),
                     tableWidget_SearchArea_RB->item(i, 10)->text().toInt());
    int id_search = SQLGetMaximumSearchID(id_chip) + 1;
    SQLInsertSearch(id_search, id_chip,
                    spinBox_StepX->value(), spinBox_StepY->value());
  }
}

void QNode::on_Button_CaptureEdge_RB_clicked() {
  int i, j;

  // Local iconic variables
  HObject ho_GrayImage_list, ho_Image, ho_TiledImage;
  HObject ho_Region1, ho_BinImage, ho_ImageCorner, ho_Corners;
  HObject ho_CornersConnected, ho_Region1Connected, ho_Region1Connected_sort;
  HObject ho_CornersConnected_sort, ho_Corner, ho_SelectedChip;
  HObject ho_chip_temp;

  // Local control variables
  // HTuple hv_yokoidou, hv_tateidou;
  HTuple hv_temporary_folder;
  HTuple hv_Tuple11, hv_i, hv_j, hv_Tuple_row, hv_Tuple_col;
  HTuple hv_Index, hv_row, hv_column, hv_radius, hv_count;
  HTuple hv_chipcount, hv_col_c, hv_row_c, hv_isEqual, hv_chipno;

  double pos_x[4096], pos_y[4096];

  SetObjectiveLensTartlet(0);
  ros::Duration(1).sleep();

  HTuple hv_homedir;

  TupleEnvironment("HOME", &hv_homedir);
  hv_temporary_folder = hv_homedir + "/images/temporary_images/";

  for (i = 0; i < numsteps_x * numsteps_y; i++) {
    hv_Tuple11[i] = -1;
  }

  GenEmptyObj(&ho_GrayImage_list);

  for (i = 0; i < numsteps_y; i++) {
    for (j = 0; j < numsteps_x; j++) {
      hv_Tuple_row[j + (i * numsteps_x)] = (i * 1016) / 8;
      if ((i % 2) == 0) {
        hv_Tuple_col[j + (i * numsteps_x)] = (j * 2034) / 8;
      } else {
        hv_Tuple_col[j + (i * numsteps_x)] = ((numsteps_x - j) * 2034) / 8;
      }
    }
  }

  for (hv_Index = (numsteps_x * numsteps_y) - 1; hv_Index >= 0;
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
  CornerResponse(ho_BinImage, &ho_ImageCorner, 11, 0.14);
  //////// Parameter
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

  /*
  QMessageBox msgBox;
  msgBox.setText(tr("Capture Origin Images at designated positions."));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Cancel) {
    HDevWindowStack::SetActive(hv_WindowHandle_originPos);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    return;
  }*/
  ros::Duration(5).sleep();

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
  double scaling_factor = (static_cast<double>(step_x)) / 2034.0 * 8.0;
  double dbuf;

  for (i = 0; i < numcorner; i++) {
    dbuf = hv_column[i];
    pos_x[i] = static_cast<int>(dbuf * scaling_factor) -
               static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_row[i];
    pos_y[i] = -static_cast<int>(dbuf * scaling_factor) +
               static_cast<int>(step_y / 2) + origin.y + 500;
  }

  currentState = Search;

  for (i = 0; i < numcorner; i++) {
    char keyname[4096];
    QTableWidgetItem* item;
    geometry_msgs::Point movpoint;

    movpoint.x = pos_x[i];
    movpoint.y = pos_y[i];
    adm2_abs_publisher.publish(movpoint);
    adm2_wait_for_stop_client.call(emp_srv);

    ros::Duration(1).sleep();
    afc_5_publisher_sc0.publish(emp_msg);
    ros::Duration(1).sleep();
    ROS_INFO("chipno:%d\n", static_cast<int>(hv_chipno[i]));
    item = tableWidget_SearchArea_RB->item(
        static_cast<int>(hv_chipno[i]) - 1, 8);

    /// Grab image and stor grabbed image to ramdisk
    GrabImage(&ho_OMImage, hv_AcqHandle);
    DispObj(ho_OMImage, hv_WindowHandleOM);
    SaveImage_jpg(temp_filename.toLocal8Bit().data());

    // Upload image file to S3 bucket
    snprintf(
        keyname, sizeof(keyname),
        "%s/%02d/edge_%06d_%06d_%06d.jpg",
        lineEdit_Folder_RB->text().toLocal8Bit().data(),
        static_cast<int>(hv_chipno[i]),
        i,
        static_cast<int>(pos_x[i]),
        static_cast<int>(pos_y[i]));
    tdmms_finder_network_support::UploadS3 sv;
    sv.request.keyname = keyname;
    finder_s3_upload_service.call(sv);

    /*
    snprintf(filename, sizeof(filename), "%s/o_%010d_%010d_%010d_%010d.jpg",
            item->text().toStdString().c_str(), i, static_cast<int>(pos_x[i]),
            static_cast<int>(pos_y[i]), static_cast<int>(0));*/

    ROS_INFO("Upload edge image to S3 bucket. Keyname: %s\n", keyname);
    // SaveImage_jpg(filename);

    int id_search = SQLGetMaximumSearchID(
        tableWidget_SearchArea_RB->item(static_cast<int>(hv_chipno[i]) - 1, 11)
            ->text().toInt());

    int id_origin_image = SQLGetMaximumEdgeID() + 1;

    SQLInsertEdgeImage(
        id_origin_image,  /// Origin image id
        id_search,        // Search ID
        tableWidget_SearchArea_RB->item(
            static_cast<int>(hv_chipno[i]) - 1, 11)->text().toInt(),  // Chip ID
        0, // Lens ID
        static_cast<int>(pos_x[i]),
        static_cast<int>(pos_y[i]),
        lineEdit_S3Bucket->text().toLocal8Bit().data(),
        keyname,
        (static_cast<std::string>(sv.response.url)).c_str());
    qApp->processEvents();
  }
  currentState = Live;
}

void QNode::on_Button_CaptureEdge_DL_clicked() {
  int i, j;

  // Local iconic variables
  HObject ho_GrayImage_list, ho_Image, ho_TiledImage;
  HObject ho_Region1, ho_BinImage, ho_ImageCorner, ho_Corners;
  HObject ho_CornersConnected, ho_Region1Connected, ho_Region1Connected_sort;
  HObject ho_CornersConnected_sort, ho_Corner, ho_SelectedChip;
  HObject ho_chip_temp;

  // Local control variables
  // HTuple hv_yokoidou, hv_tateidou;
  HTuple hv_temporary_folder;
  HTuple hv_Tuple11, hv_i, hv_j, hv_Tuple_row, hv_Tuple_col;
  HTuple hv_Index, hv_row, hv_column, hv_radius, hv_count;
  HTuple hv_chipcount, hv_col_c, hv_row_c, hv_isEqual, hv_chipno;

  double pos_x[4096], pos_y[4096];

  SetObjectiveLensTartlet(0);
  ros::Duration(1).sleep();

  HTuple hv_homedir;

  TupleEnvironment("HOME", &hv_homedir);
  hv_temporary_folder = hv_homedir + "/images/temporary_images/";

  for (i = 0; i < numsteps_x * numsteps_y; i++) {
    hv_Tuple11[i] = -1;
  }

  GenEmptyObj(&ho_GrayImage_list);

  for (i = 0; i < numsteps_y; i++) {
    for (j = 0; j < numsteps_x; j++) {
      hv_Tuple_row[j + (i * numsteps_x)] = (i * 1016) / 8;
      if ((i % 2) == 0) {
        hv_Tuple_col[j + (i * numsteps_x)] = (j * 2034) / 8;
      } else {
        hv_Tuple_col[j + (i * numsteps_x)] = ((numsteps_x - j) * 2034) / 8;
      }
    }
  }

  for (hv_Index = (numsteps_x * numsteps_y) - 1; hv_Index >= 0;
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
  CornerResponse(ho_BinImage, &ho_ImageCorner, 11, 0.14);
  //////// Parameter
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

  /*
  QMessageBox msgBox;
  msgBox.setText(tr("Capture Origin Images at designated positions."));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Cancel) {
    HDevWindowStack::SetActive(hv_WindowHandle_originPos);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    return;
  }*/
  ros::Duration(5).sleep();

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
  double scaling_factor = (static_cast<double>(step_x)) / 2034.0 * 8.0;
  double dbuf;

  for (i = 0; i < numcorner; i++) {
    dbuf = hv_column[i];
    pos_x[i] = static_cast<int>(dbuf * scaling_factor) -
               static_cast<int>(step_x / 2) + origin.x;
    dbuf = hv_row[i];
    pos_y[i] = -static_cast<int>(dbuf * scaling_factor) +
               static_cast<int>(step_y / 2) + origin.y + 500;
  }

  currentState = Search;

  for (i = 0; i < numcorner; i++) {
    char keyname[4096];
    QTableWidgetItem* item;
    geometry_msgs::Point movpoint;

    movpoint.x = pos_x[i];
    movpoint.y = pos_y[i];
    adm2_abs_publisher.publish(movpoint);
    adm2_wait_for_stop_client.call(emp_srv);

    ros::Duration(1).sleep();
    afc_5_publisher_sc0.publish(emp_msg);
    ros::Duration(1).sleep();
    ROS_INFO("chipno:%d\n", static_cast<int>(hv_chipno[i]));
    item = tableWidget_SearchArea_DL->item(static_cast<int>(hv_chipno[i]) - 1, 8);

    /// Grab image and stor grabbed image to ramdisk
    GrabImage(&ho_OMImage, hv_AcqHandle);
    DispObj(ho_OMImage, hv_WindowHandleOM);
    SaveImage_jpg(temp_filename.toLocal8Bit().data());

    // Upload image file to S3 bucket
    snprintf(
        keyname, sizeof(keyname),
        "%s/%02d/edge_%06d_%06d_%06d.jpg",
        lineEdit_Folder_DL->text().toLocal8Bit().data(),
        static_cast<int>(hv_chipno[i]),
        i,
        static_cast<int>(pos_x[i]),
        static_cast<int>(pos_y[i]));
    tdmms_finder_network_support::UploadS3 sv;
    sv.request.keyname = keyname;
    finder_s3_upload_service.call(sv);

    ROS_INFO("Upload edge image to S3 bucket. Keyname: %s\n", keyname);

    int id_search = SQLGetMaximumSearchID(
        tableWidget_SearchArea_DL->item(static_cast<int>(hv_chipno[i]) - 1, 11)
            ->text().toInt());

    int id_origin_image = SQLGetMaximumEdgeID() + 1;

    SQLInsertEdgeImage(
        id_origin_image,  /// Origin image id
        id_search,        // Search ID
        tableWidget_SearchArea_DL->item(
            static_cast<int>(hv_chipno[i]) - 1, 11)->text().toInt(),  // Chip ID
        0, // Lens ID
        static_cast<int>(pos_x[i]),
        static_cast<int>(pos_y[i]),
        lineEdit_S3Bucket->text().toLocal8Bit().data(),
        keyname,
        (static_cast<std::string>(sv.response.url)).c_str());
    qApp->processEvents();
  }
  currentState = Live;
}

bool QNode::isInside(geometry_msgs::Point rect[4], geometry_msgs::Point pnt) {
  //////////////////////////////////////////////////////////
  //// returns true if pnt is inside the rectangle defined by rect
  ///////////////////////////////////////////////////////////

  int cnt = 0;
  for (int i = 0; i < 4; ++i) {
    const double x1 = rect[(i + 1) % 4].x - rect[i].x;
    const double y1 = rect[(i + 1) % 4].y - rect[i].y;
    const double x2 = pnt.x - rect[i].x;
    const double y2 = pnt.y - rect[i].y;
    if (x1 * y2 - x2 * y1 < 0)
      ++cnt;
    else
      --cnt;
  }
  return cnt == 4 || cnt == -4;
}

void QNode::ExtractPointListFromTableWidget(
    QTableWidget* tableWidget,
    std::list<geometry_msgs::Point>* PointList,
    int row, int stepx, int stepy) {
  geometry_msgs::Point P[4];
  geometry_msgs::Point P_LeftBottom, P_RightTop;
  geometry_msgs::Point PointToSearch;
  QTableWidgetItem* item;

  PointList->clear();
  for (int i = 0; i < 4; i++) {
    item = tableWidget->item(row, i * 2);
    P[i].x = item->text().toDouble();
    item = tableWidget->item(row, i * 2 + 1);
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

void QNode::ExecuteAutofocus() {
  afc_5_publisher_sc0.publish(emp_msg);
}

void QNode::CaptureAlignmentImage(QTableWidget* tableWidget_SearchArea,
                                  int id_chiptray,
                                  QString S3_Folder) {
  int flakecount = 0;
  std::list<geometry_msgs::Point> PointList;
  QTableWidgetItem* item;
  QTableWidgetItem* item_progress;
  int row;
  tdmms_finder_network_support::UploadS3 sv;
  int tartlet_pos[] = {0, 1};
  double delay[] = {0.6, 0.3};
  currentState = Search;

  for (int i = 0 ; i < 2 ; i ++) {
    for (row = 0; row < tableWidget_SearchArea->rowCount(); row++) {
      if (tartlet_pos[i] == 0) {
        ExtractPointListFromTableWidget(tableWidget_SearchArea,
                                        &PointList, row, spinBox_StepX->value() * 4,
                                        spinBox_StepY->value() * 4);
      } else {
        ExtractPointListFromTableWidget(tableWidget_SearchArea,
                                        &PointList, row, spinBox_StepX->value() * 2,
                                        spinBox_StepY->value() * 2);
      }
      char filename[4028];
      double progress = 0.0;
      double count = 0;
      int id_chip;
      int id_search;

      SetObjectiveLensTartlet(tartlet_pos[i]);
      ros::Duration(0.5).sleep();

      adm2_abs_publisher.publish(*PointList.begin());
      adm2_wait_for_stop_client.call(emp_srv);

      ExecuteAutofocus();
      ros::Duration(1).sleep();

      ExecuteAutofocus();
      ros::Duration(1).sleep();

      item_progress = tableWidget_SearchArea->item(row, 9);

      id_chip =
          SQLGetChipID(id_chiptray,
                       tableWidget_SearchArea->item(row, 10)->text().toInt());
      id_search = SQLGetMaximumSearchID(id_chip);

      for (std::list<geometry_msgs::Point>::iterator itr = PointList.begin();
           itr != PointList.end();) {
        // Move xy stage to the next position
        adm2_abs_publisher.publish(*itr);

        // update progress
        progress = (count+1) / PointList.size();
        item_progress->setText(QString::number(progress * 100));

        // wait until the stagion has been stopped
        ros::Duration(delay[i]).sleep();

        //count
        count++;

        /// Grab image and stor grabbed image to ramdisk
        GrabImage(&ho_OMImage, hv_AcqHandle);
        DispObj(ho_OMImage, hv_WindowHandleOM);
        SaveImage_jpg(temp_filename.toLocal8Bit().data());

        // upload file to S3 server
        char keyname[4096];
        snprintf(
            keyname, sizeof(keyname),
            "%s/%02d/align_%01d_%06d_%06d_%06d.jpg",
            S3_Folder.toLocal8Bit().data(),
            row+1,
            tartlet_pos[i],
            flakecount,
            static_cast<int>(itr->x),
            static_cast<int>(itr->y));
        sv.request.keyname = keyname;
        finder_s3_upload_service.call(sv);
        SQLInsertAlignmentImage(id_search,
                                id_chip,
                                tartlet_pos[i],
                                static_cast<int>(itr->x),
                                static_cast<int>(itr->y),
                                S3_Folder.toLocal8Bit().data(),
                                keyname,
                                (static_cast<std::string>(sv.response.url)).c_str());
        flakecount++;
        itr++;
        qApp->processEvents();
      }
    }
  }
  currentState = Live;
}

void QNode::on_Button_CaptureAlignmentImage_RB_clicked() {
  CaptureAlignmentImage(tableWidget_SearchArea_RB,
                        spinBox_ChiptrayID_RB->value(),
                        lineEdit_Folder_RB->text());
}

void QNode::on_Button_CaptureAlignmentImage_DL_clicked() {
  CaptureAlignmentImage(tableWidget_SearchArea_DL,
                        spinBox_ChiptrayID_DL->value(),
                        lineEdit_Folder_DL->text());
}

void QNode::on_Button_Add_DL_clicked() {
  tableWidget_SearchArea_DL->insertRow(tableWidget_SearchArea_DL->rowCount());
  tableWidget_SearchArea_DL->resizeColumnsToContents();
  tableWidget_SearchArea_DL->resizeRowsToContents();
  QTableWidgetItem* item;
  for (int i = 0; i < 12; i++) {
    item = new QTableWidgetItem(QString("0"));
    tableWidget_SearchArea_DL->setItem(tableWidget_SearchArea_DL->rowCount() - 1, i,
                                   item);
  }
  tableWidget_SearchArea_DL->setCurrentItem(item);
}

void QNode::on_Button_Add_RB_clicked() {
  tableWidget_SearchArea_RB->insertRow(tableWidget_SearchArea_RB->rowCount());
  tableWidget_SearchArea_RB->resizeColumnsToContents();
  tableWidget_SearchArea_RB->resizeRowsToContents();
  QTableWidgetItem* item;
  for (int i = 0; i < 12; i++) {
    item = new QTableWidgetItem(QString("0"));
    tableWidget_SearchArea_RB->setItem(tableWidget_SearchArea_RB->rowCount() - 1, i,
                                   item);
  }
  tableWidget_SearchArea_RB->setCurrentItem(item);
}

void QNode::on_Button_OpenTestWindows_RB_clicked() {
  if (issettingMode) return;
  
  SetWindowAttr("window_title", "OM Image Edge");
  SetWindowAttr("background_color", "black");
  OpenWindow(0, hv_Width + hv_Width + hv_Width/2, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_Edge);

  HDevWindowStack::Push(hv_WindowHandle_Edge);
  SetWindowAttr("window_title", "OM Image H");
  SetWindowAttr("background_color", "black");
  OpenWindow(0, hv_Width + 1, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_H);
  HDevWindowStack::Push(hv_WindowHandle_H);

  SetWindowAttr("window_title", "OM Image S");
  SetWindowAttr("background_color", "black");
  OpenWindow(0, hv_Width + hv_Width/2, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_S);
  HDevWindowStack::Push(hv_WindowHandle_S);

  SetWindowAttr("window_title", "OM Image V");
  SetWindowAttr("background_color", "black");
  OpenWindow(0, hv_Width + hv_Width, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_V);
  HDevWindowStack::Push(hv_WindowHandle_V);

  SetWindowAttr("window_title", "OM Image H_th");
  SetWindowAttr("background_color", "black");
  OpenWindow(hv_Height/2, hv_Width + 1, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_H_th);
  HDevWindowStack::Push(hv_WindowHandle_H_th);

  SetWindowAttr("window_title", "OM Image S_th");
  SetWindowAttr("background_color", "black");
  OpenWindow(hv_Height/2, hv_Width + hv_Width/2, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_S_th);
  HDevWindowStack::Push(hv_WindowHandle_S_th);

  SetWindowAttr("window_title", "OM Image V_th");
  SetWindowAttr("background_color", "black");
  OpenWindow(hv_Height/2, hv_Width + hv_Width, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_V_th);
  HDevWindowStack::Push(hv_WindowHandle_V_th);

  SetWindowAttr("window_title", "OM Image ClosedRegions");
  SetWindowAttr("background_color", "black");
  OpenWindow(hv_Height/2, hv_Width + hv_Width + hv_Width/2, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_ClosedRegions);
  HDevWindowStack::Push(hv_WindowHandle_ClosedRegions);

  SetWindowAttr("window_title", "OM Image Results");
  SetWindowAttr("background_color", "black");
  OpenWindow(0, hv_Width*3, hv_Width / 2,
             hv_Height / 2, 0, "", "", &hv_WindowHandle_Result);
  HDevWindowStack::Push(hv_WindowHandle_Result);
  issettingMode = true;
}

void QNode::on_Button_CloseTestWindows_RB_clicked() {
  checkBox_TestMode_DL->setChecked(false);
  if(!issettingMode) return;
  issettingMode = false;

  ros::Duration(1).sleep();

  HDevWindowStack::SetActive(hv_WindowHandle_Edge);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_H);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_S);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_V);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_H_th);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_S_th);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_V_th);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_ClosedRegions);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

  HDevWindowStack::SetActive(hv_WindowHandle_Result);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
}

std::vector<std::string> QNode::split(std::string& input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "tdmms_finder_dl");

  if (!ros::master::check()) {
    return false;
  }

  ros::start();  // explicitly needed since our nodehandle is going out of
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
  // joy_subscriber = n.subscribe("joy", 1, &QNode::joy_Callback, this);
  adm2_wait_for_stop_client =
      n.serviceClient<std_srvs::Empty>("/adm2_master/wait_for_stop");
  adm2_vel_publisher = n.advertise<adm2::velocity>("/adm2_master/cmd_vel", 1);
  adm2_get_current_pos_client =
      n.serviceClient<adm2::GetCurrentPos>(
          "/adm2_master/get_currpos");
  start();
  return true;
}

bool QNode::init(const std::string& master_url, const std::string& host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "tdmms_finder_dl");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
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
  // joy_subscriber = n.subscribe("joy", 1, &QNode::joy_Callback, this);
  adm2_wait_for_stop_client =
      n.serviceClient<std_srvs::Empty>("/adm2_master/wait_for_stop");
  adm2_vel_publisher = n.advertise<adm2::velocity>("/adm2_master/cmd_vel", 1);
  finder_network_publisher = n.advertise<std_msgs::String>("nw_chatter", 1000);
  finder_s3_upload_service = n.serviceClient<tdmms_finder_network_support::UploadS3>("upload_s3");
  adm2_get_current_pos_client =
      n.serviceClient<adm2::GetCurrentPos>(
          "/adm2_master/get_currpos");
  start();
  return true;
}

void QNode::currpos_Callback(const geometry_msgs::Point& pnt) {
  currpoint = pnt;
}


void QNode::runDetection(void) {
  HObject ho_Image_S;
  HObject ho_Image_C;
  BitRshift(ho_OMImage, &ho_Image_S, 4);
  ConvertImageType(ho_Image_S, &ho_Image_C, "byte");
  WriteImage(ho_Image_C, "jpeg", 0, "/ramdisk/tmp.jpg");

  char commands[4096];
  //////////////////////////////////////////////////////////
  // Run Inference
  //////////////////////////////////////////////////////////
  snprintf(
      commands, sizeof(commands),
      "python3 "
      "%s/tdmms_ws/src/tdmms_finder/tdmms_finder_dl/dl_infer.py "
      "/ramdisk/tmp.jpg",
      QDir::homePath().toStdString().c_str() );
  system(commands);

  ////////////////////////////////////////////////////////////
  /// Read results of inference -- scores
  ////////////////////////////////////////////////////////////
  std::ifstream f;
  f.open("/ramdisk/tmp_scores.csv");
  if (!f.is_open()) {
    std::cerr << "error: file open failed.\n";
    return;
  }
  std::string line_score;
  std::getline(f, line_score);
  std::vector<std::string> strvec_score = split(line_score, ',');

  std::vector<double> scores;
  for (int i = 0; i < strvec_score.size(); i++) {
    scores.push_back(std::atof(strvec_score.at(i).c_str()));
  }
  f.close();

  ///////////////////////////////////////////////////////////////
  /// Read results of inference -- rois
  ///////////////////////////////////////////////////////////////
  f.open("/ramdisk/tmp_roi.csv");
  if (!f.is_open()) {
    std::cerr << "error: file open failed.\n";
    return;
  }
  std::string line, val;  
  std::vector<int> array; 

  while (std::getline(f, line)) { 
    std::vector<int> v;           
    std::stringstream s(line);    
    while (getline(s, val, ',')) {
      std::istringstream iss1(val);
      int val_int;
      iss1 >> val_int;
      array.push_back(val_int); 
    }
  }
  f.close();

  std::vector<int> roi_y1, roi_x1, roi_y2, roi_x2;
  for (std::vector<int>::const_iterator it = array.begin(), e = array.end();
       it != e; ++it) {
    roi_y1.push_back(*it);
    roi_x1.push_back(*(++it));
    roi_y2.push_back(*(++it));
    roi_x2.push_back(*(++it));
  }

  //////////////////////////////////////////////////////////
  /// Read results of inference -- masks
  //////////////////////////////////////////////////////////
  f.open("/ramdisk/tmp_masks.csv");
  if (!f.is_open()) {
    std::cerr << "error: file open failed.\n";
    return;
  }
  std::string line2, val2; 
  std::vector<int> array2; 

  std::vector<int> array_obj_id;
  std::vector<int> array_mask_id;
  std::vector<int> array_mask_row;
  std::vector<int> array_mask_class;
  std::vector<int> array_mask_column;

  while (std::getline(f, line2)) {
    std::vector<std::string> strvec = split(line2, ',');
    array_obj_id.push_back(std::atoi(strvec.at(0).c_str()));
    array_mask_id.push_back(std::atoi(strvec.at(1).c_str()));
    array_mask_class.push_back(std::atoi(strvec.at(2).c_str()));
    array_mask_column.push_back(std::atoi(strvec.at(3).c_str()));
    array_mask_row.push_back(std::atoi(strvec.at(4).c_str()));
  }
  f.close();

  /////////////////////////////////////////////////////////
  /// Display results of inference
  /////////////////////////////////////////////////////////
  if(!checkBox_Enhance_DL->isChecked()){
    DispObj(ho_OMImage, hv_WindowHandleOM);
    for (int i = 0; i < array_obj_id.size();) {
      int obj_id = array_obj_id.at(i);
      int mask_id = array_mask_id.at(i);
      int j = 0;
      HTuple rows;
      HTuple columns;
      do {
        rows[j] = array_mask_row.at(i);
        columns[j] = array_mask_column.at(i);
        j++;
        i++;
        if (i == array_obj_id.size()) break;
      } while (obj_id == array_obj_id.at(i) && mask_id == array_mask_id.at(i));

      std::string label;
      switch (array_mask_class.at(i - 1)) {
        case 1:
          SetColor(hv_WindowHandleOM, "red");
          label = "mono";
          break;
        case 2:
          SetColor(hv_WindowHandleOM, "green");
          label = "few";
          break;
        case 3:
          SetColor(hv_WindowHandleOM, "blue");
          label = "thick";
          break;
        default:
          SetColor(hv_WindowHandleOM, "black");
          std::stringstream temp;
          temp << array_mask_class.at(i-1);
          label = temp.str();
          // SetColor(hv_WindowHandleOM, "white");
          // label = "unknown";
          break;
      }
      int lineWidth = 4;
      SetLineWidth(hv_WindowHandleOM, lineWidth);

      DispPolygon(hv_WindowHandleOM, rows, columns);

      DispLine(hv_WindowHandleOM,
               roi_y1.at(obj_id) - lineWidth, roi_x1.at(obj_id) - lineWidth,
               roi_y1.at(obj_id) - lineWidth, roi_x2.at(obj_id) + lineWidth);
      DispLine(hv_WindowHandleOM,
               roi_y1.at(obj_id) - lineWidth, roi_x2.at(obj_id) + lineWidth,
               roi_y2.at(obj_id) + lineWidth, roi_x2.at(obj_id) + lineWidth);
      DispLine(hv_WindowHandleOM,
               roi_y2.at(obj_id) + lineWidth, roi_x2.at(obj_id) + lineWidth,
               roi_y2.at(obj_id) + lineWidth, roi_x1.at(obj_id) - lineWidth);
      DispLine(hv_WindowHandleOM,
               roi_y2.at(obj_id) + lineWidth, roi_x1.at(obj_id) - lineWidth,
               roi_y1.at(obj_id) - lineWidth, roi_x1.at(obj_id) - lineWidth);
      halcon_bridge::disp_message(hv_WindowHandleOM, label.c_str(), "window",
                                  roi_y1.at(obj_id) - 20, roi_x1.at(obj_id),
                                  "white", "false");
      std::ostringstream oss;
      oss << scores.at(obj_id);
      halcon_bridge::disp_message(hv_WindowHandleOM, oss.str().c_str(),
                                  "window",
                                  roi_y1.at(obj_id) - 35, roi_x1.at(obj_id),
                                  "white", "false");
    }
  } else {
    DispObj(ho_OMImage, hv_WindowHandleOM);
    for (int i = 0; i < array_obj_id.size();) {
      int obj_id = array_obj_id.at(i);
      int mask_id = array_mask_id.at(i);
      int j = 0;
      HTuple rows;
      HTuple columns;
      do {
        rows[j] = array_mask_row.at(i);
        columns[j] = array_mask_column.at(i);
        j++;
        i++;
        if (i == array_obj_id.size()) break;
      } while (obj_id == array_obj_id.at(i) && mask_id == array_mask_id.at(i));
      //rows[j+1] = rows[0];
      //columns[j+1] = columns[0];
      std::string label;
      std::string colorname;
      switch (array_mask_class.at(i - 1)) {
        case 1:
          label = "Mono";
          colorname = "red";
          SetColor(hv_WindowHandleOM, colorname.c_str());
          break;
        case 2:
          label = "Few";
          colorname = "green";
          SetColor(hv_WindowHandleOM, colorname.c_str());
          break;
        case 3:
          label = "Thick";
          colorname = "blue";
          SetColor(hv_WindowHandleOM, colorname.c_str());
          break;
        default:
          SetColor(hv_WindowHandleOM, "black");
          std::stringstream temp;
          temp << array_mask_class.at(i-1);
          label = temp.str();
          //SetColor(hv_WindowHandleOM, "white");
          //label = "unknown";
          break;
      }
      int linewidth = 2;
      HObject ho_RegionPolygon, ho_RegionBBox1, ho_RegionBBox2, ho_RegionBBox3, ho_RegionBBox4;
      GenRegionPolygonFilled(&ho_RegionPolygon, rows, columns);

      int overlay_color[] = {128, 0, 0};
      HObject ho_Image_Mask_R, ho_Image_Mask_G, ho_Image_Mask_B;
      GenImageConst(&ho_Image_Mask_R, "byte", hv_Width, hv_Height);
      GenImageConst(&ho_Image_Mask_G, "byte", hv_Width, hv_Height);
      GenImageConst(&ho_Image_Mask_B, "byte", hv_Width, hv_Height);

      PaintRegion(ho_RegionPolygon, ho_Image_Mask_R,
                  &ho_Image_Mask_R, overlay_color[0], "fill");
      PaintRegion(ho_RegionPolygon, ho_Image_Mask_G,
                  &ho_Image_Mask_G, overlay_color[1], "fill");
      PaintRegion(ho_RegionPolygon, ho_Image_Mask_B,
                  &ho_Image_Mask_B, overlay_color[2], "fill");

      HObject ho_Image_Mask_Comp;
      Compose3(ho_Image_Mask_R, ho_Image_Mask_G,
               ho_Image_Mask_B, &ho_Image_Mask_Comp);
      HObject ho_ImageRes;
      AddImage(ho_Image_C, ho_Image_Mask_Comp, &ho_ImageRes, 1, 0);

      //DispObj(ho_OMImage, hv_WindowHandleOM);

      int lineWidth = 3;
      SetLineWidth(hv_WindowHandleOM, lineWidth);
      DispPolygon(hv_WindowHandleOM, rows, columns);

      DispLine(hv_WindowHandleOM,
               roi_y1.at(obj_id) - lineWidth*2,
               roi_x1.at(obj_id) - lineWidth*2,
               roi_y1.at(obj_id) - lineWidth*2,
               roi_x2.at(obj_id) + lineWidth*2);
      DispLine(hv_WindowHandleOM,
               roi_y1.at(obj_id) - lineWidth*2,
               roi_x2.at(obj_id) + lineWidth*2,
               roi_y2.at(obj_id) + lineWidth*2,
               roi_x2.at(obj_id) + lineWidth*2);
      DispLine(hv_WindowHandleOM,
               roi_y2.at(obj_id) + lineWidth*2,
               roi_x2.at(obj_id) + lineWidth*2,
               roi_y2.at(obj_id) + lineWidth*2,
               roi_x1.at(obj_id) - lineWidth*2);
      DispLine(hv_WindowHandleOM,
               roi_y2.at(obj_id) + lineWidth*2,
               roi_x1.at(obj_id) - lineWidth*2,
               roi_y1.at(obj_id) - lineWidth*2,
               roi_x1.at(obj_id) - lineWidth*2);

      halcon_bridge::set_display_font(hv_WindowHandleOM, 30, "mono", "true", "false");
      std::ostringstream oss;
      oss << label;
      oss << " ";
      oss << scores.at(obj_id);
      halcon_bridge::disp_message(hv_WindowHandleOM, oss.str().c_str(), "window",
                                  roi_y1.at(obj_id) - 40, roi_x1.at(obj_id),
                                  colorname.c_str(), "false");
    }
  }
}

void QNode::run() {
  ////////////////////////////////
  //// Main loop
  ////////////////////////////////
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    tdmms_finder_network_support::UploadS3 sv;
    sv.request.keyname = "abc.jpg";
    if (currentState == Live) {
      GrabImage(&ho_OMImage, hv_AcqHandle);
      if (checkBox_TestMode_DL->isChecked()) {
        runDetection();
      } else if (checkBox_TestMode_RB->isChecked()) {
        runRBDetection(0, NULL, 0, 0, true, issettingMode);
      } else {
        // Display Optical Micrsocope Image
        DispObj(ho_OMImage, hv_WindowHandleOM);
      }
    } else if (currentState == Search) {
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
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

}  // namespace tdmms_finder_dl
