/**
 * @file /include/tdmms_autostamp_camera_viewer_guided/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tdmms_autostamp_camera_viewer_guided_QNODE_HPP_
#define tdmms_autostamp_camera_viewer_guided_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QTableWidget>
#include <QCheckBox>
#include <sensor_msgs/Image.h>
#include <QLineEdit>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>
#include <QSlider>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_autostamp_camera_viewer_guided {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char **argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();
  QSpinBox *spinBox_Xofs;
  QSpinBox *spinBox_Yofs;
  QSpinBox *spinBox_Edges_High;
  QSpinBox *spinBox_Edges_Low;
  QDoubleSpinBox *doubleSpinBox_Alpha;
  QLineEdit *lineEdit_DataFolder;
  QTableWidget *tableWidget_Edges;
  QCheckBox *checkBox_ExtractEdge;
  QSlider *horizontalSlider_Xofs;
  QSlider *horizontalSlider_Yofs;
  /*********************
  ** Logging
  **********************/
  enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);
  void imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image);
 public
Q_SLOTS:
  void on_SetFolder_clicked(bool check);
  void on_SaveFile_clicked(bool check);
  void on_CheckAll_clicked(bool check);
  void on_UncheckAll_clicked(bool check);
  void on_Add_clicked(bool check);
  void on_Delete_clicked(bool check);
  void horizontalSlider_Xofs_valueChanged(int value);
  void horizontalSlider_Yofs_valueChanged(int value);
  void selectionChanged();

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

 private:
  void reloadEdgeInformation();
  void drawEdge();
  void loadFile();

  int count;
  int init_argc;
  char **init_argv;
  bool edge_mutex;
  bool image_mutex;

  ros::Publisher chatter_publisher;
  ros::Subscriber image_subscriber;
  ros::ServiceClient autostamp_camera_streamer_capture_timestamp;

  QStringListModel logging_model;

  HalconCpp::HXLDCont h_Edges;
  HalconCpp::HWindow w;
  HalconCpp::HImage h_Image;
  HalconCpp::HObject ho_Edges_tuple[500];
};

}  // namespace tdmms_autostamp_camera_viewer_guided

#endif /* tdmms_autostamp_camera_viewer_guided_QNODE_HPP_ */
