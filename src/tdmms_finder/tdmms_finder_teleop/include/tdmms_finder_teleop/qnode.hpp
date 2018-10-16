/**
 * @file /include/tdmms_finder_teleop/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef tdmms_finder_teleop_QNODE_HPP_
#define tdmms_finder_teleop_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <QSpinBox>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_finder_teleop {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QSpinBox *Light_Intensity_SpinBox;
  QSpinBox *Revolver_Pos_SpinBox;
  QSpinBox *Pos_X_SpinBox;
  QSpinBox *Pos_Y_SpinBox;
  QSpinBox *Step_SpinBox;
  QSpinBox *Filter_Pos_SpinBox;
  QSpinBox *Exposure_SpinBox;
  QSpinBox *Vel_High_SpinBox;
  QSpinBox *Vel_Low_SpinBox;
  QSpinBox *Accl_SpinBox;

  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();
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

  QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg);
  void joyCallback(const sensor_msgs::Joy &joy_msg);
  void adm2Callback(const geometry_msgs::Point &currpoint);

 Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

 public Q_SLOTS:
  void on_Light_Intensity_SpinBox_valueChanged();
  void on_Light_ON_Button_clicked(bool check);
  void on_Light_OFF_Button_clicked(bool check);
  void on_Step_SpinBox_valueChanged();
  void on_Exposure_SpinBox_valueChanged();
  void on_Vel_SpinBox_valueChanged();

 private:
  int init_argc;
  char** init_argv;
  // Communication Nodes for Revolver Controller
  ros::Publisher lv_ncnt_n_publisher;
  // Communication Nodes for Light Illumination Unit
  ros::Publisher nic_100a_on_publisher;
  ros::Publisher nic_100a_off_publisher;
  ros::Publisher nic_100a_inten_publisher;
  // Communication Nodes for XY Stage Controller
  ros::Publisher adm2_publisher;
  ros::Publisher adm2_jog_publisher;
  ros::Publisher adm2_step_publisher;
  ros::Publisher adm2_stop_publisher;
  ros::Publisher adm2_vel_publisher;
  ros::Subscriber adm2_subscriber;
  // Communication Nodes for Autofocus Controller
  ros::Publisher afc_5_publisher;
  ros::Publisher afc_5_publisher_sc0;
  // Communication Nodes for Basler CMOS Camera
  ros::Publisher ace2000_exp_publisher;
  // Communication Nodes for Optical Filter Wheel Controller
  ros::Publisher bsc201_home_publisher;
  ros::Publisher bsc201_abs_publisher;
  // Communication Nodes for joystick interface
  ros::Subscriber joy_subscriber;

  QStringListModel logging_model;
  unsigned int revolver_pos;
  unsigned int filter_pos;

  bool jog_moving;
};

}  // namespace tdmms_finder_teleop

#endif /* tdmms_finder_teleop_QNODE_HPP_ */
