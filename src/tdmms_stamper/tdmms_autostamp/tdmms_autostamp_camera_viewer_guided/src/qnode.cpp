
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
#include "../include/tdmms_autostamp_camera_viewer_guided/qnode.hpp"
#include <time.h>
#include <sys/time.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>
#include <QFileDialog>
#include <QString>
#include <QTextStream>
#include <QDir>
#include <tdmms_autostamp_camera_streamer/CaptureImageTimestamp.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_autostamp_camera_viewer_guided {
using namespace HalconCpp;

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv)
    : init_argc(argc),
      init_argv(argv),
      w(0, 0, 744 * 1.5, 480 * 1.5, 0, "", ""){

  QString initFile;
  initFile = QDir::homePath() + "/img744x480.jpg";
  ReadImage(&h_Image, initFile.toLatin1().data());
  /////////////////////////
  // Initialize Parameters
  /////////////////////////
  w.SetPart(0, 0, 480, 744);
  edge_mutex = false;
  image_mutex = false;
  count = 0;
}

QNode::~QNode() {
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  on_SaveFile_clicked(true);
  wait();
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "tdmms_autostamp_camera_viewer_guided");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  image_subscriber = n.subscribe("/autostamp_camera_streamer/image_main", 100,
                                 &QNode::imageStreamCallback, this);
  autostamp_camera_streamer_capture_timestamp =
      n.serviceClient<tdmms_autostamp_camera_streamer::CaptureImageTimestamp>(
          "/autostamp_camera_streamer/capture_timestamp");

  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "tdmms_autostamp_camera_viewer_guided");
  if (!ros::master::check()) {
    return false;
  }
  loadFile();
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  image_subscriber = n.subscribe("/autostamp_camera_streamer/image_main", 100,
                                 &QNode::imageStreamCallback, this);
  start();
  return true;
}

void QNode::imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image) {
  if (!image_mutex) {
    halcon_bridge::toHImage(Image, &h_Image);
  }
  h_Image.DispColor(w);
  ////////////////////////////
  // Draw Guide Lines
  ///////////////////////////
  HTuple hv_Width, hv_Height, hv_Offset;
  hv_Width = 744;
  hv_Height = 480;
  hv_Offset = 55;

  if (checkBox_DisplayGrid->isChecked()) {
    w.SetColor("red");
    w.DispLine((HTuple)0, hv_Width / 2, hv_Height, hv_Width / 2);
    w.DispLine(hv_Height / 2, (HTuple)0, hv_Height / 2, hv_Width);
    w.SetColor("red");
    w.DispLine((HTuple)0, hv_Width / 2 - hv_Offset, hv_Height,
               hv_Width / 2 - hv_Offset);
    w.DispLine(hv_Height / 2 - hv_Offset, (HTuple)0, hv_Height / 2 - hv_Offset,
               hv_Width);
    w.DispLine((HTuple)0, hv_Width / 2 + hv_Offset, hv_Height,
               hv_Width / 2 + hv_Offset);
    w.DispLine(hv_Height / 2 + hv_Offset, (HTuple)0, hv_Height / 2 + hv_Offset,
               hv_Width);
  }
  /////////////////////////
  // Extract Edge if Checkbox is enabled
  ////////////////////////
  if (checkBox_ExtractEdge->isChecked()) {
    if (!edge_mutex) {
      try {
        image_mutex = true;
        h_Edges = h_Image.Rgb1ToGray().EdgesSubPix(
            "canny", doubleSpinBox_Alpha->value(), spinBox_Edges_Low->value(),
            spinBox_Edges_High->value());
        image_mutex = false;
        h_Edges.DispObj(w);
      }
      catch (HalconCpp::HException &HDevExpDefaultException) {
      }
    }
  }
  if (!edge_mutex) {
    drawEdge();
  }
}

void QNode::run() {
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to
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

void QNode::on_SetFolder_clicked(bool check) {
  QFileDialog::Options options =
      QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly;

  QString strDir = QFileDialog::getExistingDirectory(
      0, tr("Folder to Store Edge Data"), QDir::homePath() + "/tdmms_data/log",
      options);

  if (!strDir.isEmpty()) {
    lineEdit_DataFolder->setText(strDir);
  }
}

void QNode::loadFile() {
  QString fileName = lineEdit_DataFolder->text() + "/lines.edg";
  QFileInfo check_file(fileName);
  if (check_file.exists()) {
    QFile file(fileName);
    QString data;
    QStringList rowOfData;
    QStringList rowData;
    data.clear();
    rowOfData.clear();
    rowData.clear();

    if (file.open(QFile::ReadOnly)) {
      data = file.readAll();
      rowOfData = data.split("\n");
      file.close();
    }

    tableWidget_Edges->clear();
    tableWidget_Edges->setRowCount(0);
    for (int x = 0; x < rowOfData.size() - 1; x++) {
      tableWidget_Edges->insertRow(tableWidget_Edges->rowCount());
      rowData = rowOfData.at(x).split(";");
      for (int y = 0; y < rowData.size(); y++) {
        QTableWidgetItem *item;
        item = new QTableWidgetItem(QString(rowData[y]));
        tableWidget_Edges->setItem(tableWidget_Edges->rowCount() - 1, y, item);
        if (y == 0) tableWidget_Edges->item(x, 0)->setCheckState(Qt::Checked);
      }
    }
  }
  tableWidget_Edges->resizeColumnsToContents();
  tableWidget_Edges->resizeRowsToContents();
  reloadEdgeInformation();
}

void QNode::on_SaveFile_clicked(bool check) {
  QFileDialog::Options options;
  QString strSelectedFilter;

  if (!lineEdit_DataFolder->text().isEmpty()) {
    QString strDir = lineEdit_DataFolder->text();
    QString strFName = strDir + "/lines.edg";
    if (!strFName.isEmpty()) {
      QFile f(strFName);
      if (f.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream data(&f);
        QStringList strList;
        for (int r = 0; r < tableWidget_Edges->rowCount(); ++r) {
          strList.clear();
          for (int c = 0; c < tableWidget_Edges->columnCount(); ++c) {
            strList << tableWidget_Edges->item(r, c)->text();
          }
          data << strList.join(";") + "\n";
        }
        f.close();
      }
    }
  }
}

void QNode::on_CheckAll_clicked(bool check) {
  for (int i = 0; i < tableWidget_Edges->rowCount(); i++) {
    tableWidget_Edges->item(i, 0)->setCheckState(Qt::Checked);
  }
}

void QNode::on_UncheckAll_clicked(bool check) {
  for (int i = 0; i < tableWidget_Edges->rowCount(); i++) {
    tableWidget_Edges->item(i, 0)->setCheckState(Qt::Unchecked);
  }
}

void QNode::on_Add_clicked(bool check) {
  if (lineEdit_DataFolder->text().isEmpty()) return;

  //////////////////
  // Capture Image
  /////////////////
  tdmms_autostamp_camera_streamer::CaptureImageTimestamp cap_timestamp;
  cap_timestamp.request.folder = static_cast<const char *>(
      lineEdit_DataFolder->text().toStdString().c_str());
  autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);

  QTableWidgetItem *item;
  QString filename = lineEdit_DataFolder->text();
  struct timeval myTime;
  struct tm *time_st;
  char buffer[1024];
  gettimeofday(&myTime, NULL);
  time_st = localtime(&myTime.tv_sec);
  snprintf(buffer, sizeof(buffer), "/%d-%02d-%02d-%02d-%02d-%02d.%06d.%04d.dxf",
           time_st->tm_year + 1900, time_st->tm_mon + 1, time_st->tm_mday,
           time_st->tm_hour, time_st->tm_min, time_st->tm_sec,
           static_cast<int>(myTime.tv_usec), count);
  filename = filename + buffer;
  count++;

  try {
    //////////////////////////////////////////////
    //// mutual execution for ho_Edges
    //////////////////////////////////////////////
    edge_mutex = true;
    ros::Duration(0.3).sleep();
    h_Edges.WriteContourXldDxf(filename.toStdString().c_str());
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    ROS_ERROR("Could not Save Edge Information");
    edge_mutex = false;
    return;
  }

  tableWidget_Edges->insertRow(tableWidget_Edges->rowCount());
  tableWidget_Edges->resizeColumnsToContents();
  tableWidget_Edges->resizeRowsToContents();

  for (int i = 0; i < 4; i++) {
    item = new QTableWidgetItem(QString("0"));
    item->setFlags(item->flags() & ~(Qt::ItemIsDropEnabled));
    item->setFlags(item->flags() | Qt::ItemIsSelectable);
    tableWidget_Edges->setItem(tableWidget_Edges->rowCount() - 1, i, item);
  }

  tableWidget_Edges->setCurrentItem(item);
  int row = tableWidget_Edges->rowCount() - 1;
  tableWidget_Edges->item(row, 0)->setCheckState(Qt::Checked);
  tableWidget_Edges->item(row, 1)->setText(tr("0"));
  tableWidget_Edges->item(row, 2)->setText(tr("0"));
  tableWidget_Edges->item(row, 3)->setText(filename);
  tableWidget_Edges->setCurrentItem(tableWidget_Edges->item(row, 0));

  reloadEdgeInformation();
  on_SaveFile_clicked(true);
  edge_mutex = false;
}

void QNode::reloadEdgeInformation() {
  HXLDCont h_Edges_temp;
  for (int i = 0; i < tableWidget_Edges->rowCount(); i++) {
    h_Edges_temp.ReadContourXldDxf(
        HTuple(tableWidget_Edges->item(i, 3)->text().toStdString().c_str()),
        HTuple(), HTuple());
    ho_Edges_tuple[i] = h_Edges_temp;
  }
}

void QNode::drawEdge() {
  w.SetColor("white");
  for (int i = 0; i < tableWidget_Edges->rowCount(); i++) {
    if (tableWidget_Edges->item(i, 0)->checkState() == Qt::Checked) {
      HObject ho_Edges_transformed;
      HTuple hv_xofs, hv_yofs;
      HTuple hv_Matrix1, hv_Matrix2;
      hv_xofs = HTuple(tableWidget_Edges->item(i, 1)->text().toInt());
      hv_yofs = HTuple(tableWidget_Edges->item(i, 2)->text().toInt());
      HomMat2dIdentity(&hv_Matrix1);
      HomMat2dTranslate(hv_Matrix1, hv_yofs, hv_xofs, &hv_Matrix2);
      AffineTransContourXld(ho_Edges_tuple[i], &ho_Edges_transformed,
                            hv_Matrix2);
      ho_Edges_transformed.DispObj(w);
    }
  }
}

void QNode::on_Delete_clicked(bool check) {
  tableWidget_Edges->clearSelection();
  tableWidget_Edges->removeRow(tableWidget_Edges->currentRow());
  reloadEdgeInformation();
  on_SaveFile_clicked(true);
}

void QNode::horizontalSlider_Xofs_valueChanged(int value) {
  if (tableWidget_Edges->rowCount() == 0) return;
  int row = tableWidget_Edges->currentRow();
  tableWidget_Edges->item(row, 1)
      ->setText(QString::number(horizontalSlider_Xofs->value()));
}
void QNode::horizontalSlider_Yofs_valueChanged(int value) {
  if (tableWidget_Edges->rowCount() == 0) return;
  int row = tableWidget_Edges->currentRow();
  tableWidget_Edges->item(row, 2)
      ->setText(QString::number(horizontalSlider_Yofs->value()));
}

void QNode::selectionChanged() {
  if (tableWidget_Edges->rowCount() == 0) return;
  int row = tableWidget_Edges->currentRow();
  horizontalSlider_Xofs->setValue(
      tableWidget_Edges->item(row, 1)->text().toInt());
  horizontalSlider_Yofs->setValue(
      tableWidget_Edges->item(row, 2)->text().toInt());
}

}  // namespace tdmms_autostamp_camera_viewer_guided
