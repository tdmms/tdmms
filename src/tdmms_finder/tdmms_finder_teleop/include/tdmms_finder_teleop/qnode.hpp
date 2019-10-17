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
#include <QRadioButton>

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
  QRadioButton *radioButton_LeftTop;
  QRadioButton *radioButton_CenterTop;
  QRadioButton *radioButton_RightTop;
  QRadioButton *radioButton_LeftCenter;
  QRadioButton *radioButton_Center;
  QRadioButton *radioButton_RightCenter;
  QRadioButton *radioButton_LeftBottom;
  QRadioButton *radioButton_CenterBottom;
  QRadioButton *radioButton_RightBottom;

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
  void on_Intensity_Descend_Button_clicked(bool check);
  
  void on_Move_01_clicked(bool check);
  void on_Move_02_clicked(bool check);
  void on_Move_03_clicked(bool check);
  void on_Move_04_clicked(bool check);
  void on_Move_05_clicked(bool check);
  void on_Move_06_clicked(bool check);
  void on_Move_07_clicked(bool check);
  void on_Move_08_clicked(bool check);
  void on_Move_09_clicked(bool check);
  void on_Move_10_clicked(bool check);
  void on_Move_11_clicked(bool check);
  void on_Move_12_clicked(bool check);
  void on_Move_13_clicked(bool check);
  void on_Move_14_clicked(bool check);
  void on_Move_15_clicked(bool check);
  void on_Move_16_clicked(bool check);
  void on_Move_17_clicked(bool check);
  void on_Move_18_clicked(bool check);
  void on_Move_19_clicked(bool check);
  void on_Move_20_clicked(bool check);
  void on_Move_21_clicked(bool check);
  void on_Move_22_clicked(bool check);
  void on_Move_23_clicked(bool check);
  void on_Move_24_clicked(bool check);
  void on_Move_25_clicked(bool check);
  void on_Move_26_clicked(bool check);
  void on_Move_27_clicked(bool check);
  void on_Move_28_clicked(bool check);
  void on_Move_29_clicked(bool check);
  void on_Move_30_clicked(bool check);
  void on_Move_31_clicked(bool check);
  void on_Move_32_clicked(bool check);
  void on_Move_33_clicked(bool check);
  void on_Move_34_clicked(bool check);
  void on_Move_35_clicked(bool check);
  void on_Move_36_clicked(bool check);

  void on_HP_clicked(bool check);
  void MoveTo(int pocketNo);
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
  ros::Publisher adm2_abs_publisher;
  ros::Publisher adm2_stop_publisher;
  ros::Publisher adm2_vel_publisher;
  ros::Publisher adm2_home_publisher;

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
