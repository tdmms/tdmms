/**
 * @file /include/tdmms_stamper_teleop/qnode.hpp
 *
 * @brief Communications central!
 *
 
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tdmms_stamper_teleop_QNODE_HPP_
#define tdmms_stamper_teleop_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <keyence_master/Status_msg.h>

#include <QLCDNumber>
#include <QSpinBox>
#include <QPushButton>
#include <QThread>
#include <QStringListModel>

#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace tdmms_stamper_teleop {

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

  QLCDNumber *lcdNumber_Temperature;
  QLCDNumber *lcdNumber_Load;
  QLCDNumber *lcdNumber_Temperature_sv;
  QLCDNumber *lcdNumber_Sample_X;
  QLCDNumber *lcdNumber_Sample_Y;
  QLCDNumber *lcdNumber_OM_X;
  QLCDNumber *lcdNumber_OM_Y;

  QSpinBox *spinBox_Temperature;
  QSpinBox *spinBox_Light_Inten;
  QSpinBox *spinBox_Sample_Theta_Spd;
  QSpinBox *spinBox_Sample_XY_Spd;
  QSpinBox *spinBox_Sample_Z_Spd;
  QSpinBox *spinBox_OM_XY_Spd;
  QSpinBox *spinBox_OM_Z_Spd;
  QSpinBox *spinBox_Auto_Stamp_Load;
  QSpinBox *spinBox_Auto_Stamp_Up_Speed;
  QSpinBox *spinBox_Auto_Stamp_Down_Speed;
  QSpinBox *spinBox_Auto_Stamp_Time;
  QSpinBox *spinBox_Sample_XY_STP;
  QSpinBox *spinBox_Sample_Z_STP;
  QSpinBox *spinBox_OM_XY_STP;

  QPushButton *pushButton_OM_Back;
  QPushButton *pushButton_OM_Forward;
  QPushButton *pushButton_Mask_Back;
  QPushButton *pushButton_Mask_Forward;
  QPushButton *pusuButton_ESD_Back;
  QPushButton *pushButton_ESD_Forward;

  QPushButton *pushButton_Sample_Blow_On;
  QPushButton *pushButton_Sample_Blow_Off;
  QPushButton *pushButton_Sample_Cool_On;
  QPushButton *pushButton_Sample_Cool_Off;
  QPushButton *pushButton_Sample_Vacuum_On;
  QPushButton *pushButton_Sample_Vacuum_Off;
  QPushButton *pushButton_Mask_Chuck_On;
  QPushButton *pushButton_Mask_Chuck_Off;
  QPushButton *pushButton_Stage_Lock_On;
  QPushButton *pushButton_Stage_Lock_Off;

  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);
  void currtemp_Callback(const std_msgs::UInt8 &temp);
  void currtemp_sv_Callback(const std_msgs::UInt8 &temp);
  void currload_Callback(const std_msgs::Float32 &load);
  void currpos_sample_xy_Callback(const geometry_msgs::Point &currpoint);
  void currpos_om_xy_Callback(const geometry_msgs::Point &currpoint);
  void keyence_Callback(const keyence_master::Status_msg &stat);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

public Q_SLOTS:
  void on_OM_Forward_Button_clicked(bool check);
  void on_OM_Back_Button_clicked(bool check);
  void on_Mask_Forward_Button_clicked(bool check);
  void on_Mask_Back_Button_clicked(bool check);
  void on_Sample_Cool_On_Button_clicked(bool check);
  void on_Sample_Cool_Off_Button_clicked(bool check);
  void on_Sample_Blow_Off_Button_clicked(bool check);
  void on_Sample_Blow_On_Button_clicked(bool check);
  void on_Stage_Lock_Off_Button_clicked(bool check);
  void on_Stage_Lock_On_Button_clicked(bool check);
  void on_Sample_Vacuum_Off_Button_clicked(bool check);
  void on_Sample_Vacuum_On_Button_clicked(bool check);
  void on_Mask_Chuck_On_Button_clicked(bool check);
  void on_Mask_Chuck_Off_Button_clicked(bool check);
  void on_Loadcell_ZeroSet_Button_clicked(bool check);
  void on_Temperature_valueChanged();

  void on_Sample_XY_X_Minus_pressed();
  void on_Sample_XY_X_Minus_released();
  void on_Sample_XY_X_Plus_pressed();
  void on_Sample_XY_X_Plus_released();
  void on_Sample_XY_X_Plus_Stp_clicked(bool check);
  void on_Sample_XY_X_Minus_Stp_clicked(bool check);

  void on_Sample_XY_Y_Minus_pressed();
  void on_Sample_XY_Y_Minus_released();
  void on_Sample_XY_Y_Plus_pressed();
  void on_Sample_XY_Y_Plus_released();
  void on_Sample_XY_Center_clicked(bool check);
  void on_Sample_XY_Y_Plus_Stp_clicked(bool check);
  void on_Sample_XY_Y_Minus_Stp_clicked(bool check);

  void on_Sample_Z_Minus_pressed();
  void on_Sample_Z_Minus_released();
  void on_Sample_Z_Plus_pressed();
  void on_Sample_Z_Plus_released();
  void on_Sample_Z_Home_clicked(bool check);
  void on_Sample_Z_Plus_Stp_clicked(bool check);
  void on_Sample_Z_Minus_Stp_clicked(bool check);
  void on_Sample_Z_Point0_clicked(bool check);
  void on_Sample_Z_Point1_clicked(bool check);
  void on_Sample_Z_Point2_clicked(bool check);

  void on_Sample_Theta_Minus_pressed();
  void on_Sample_Theta_Minus_released();
  void on_Sample_Theta_Plus_pressed();
  void on_Sample_Theta_Plus_released();
  void on_Sample_Theta_Center_clicked(bool check);

  void on_OM_XY_X_Minus_pressed();
  void on_OM_XY_X_Minus_released();
  void on_OM_XY_X_Plus_pressed();
  void on_OM_XY_X_Plus_released();
  void on_OM_XY_X_Plus_Stp_clicked(bool check);
  void on_OM_XY_X_Minus_Stp_clicked(bool check);

  void on_OM_XY_Y_Minus_pressed();
  void on_OM_XY_Y_Minus_released();
  void on_OM_XY_Y_Plus_pressed();
  void on_OM_XY_Y_Plus_released();
  void on_OM_XY_Center_clicked(bool check);
  void on_OM_XY_Y_Plus_Stp_clicked(bool check);
  void on_OM_XY_Y_Minus_Stp_clicked(bool check);

  void on_OM_Revolv_Next_clicked(bool check);
  void on_OM_Revolv_Prev_clicked(bool check);
  void on_OM_z_U1_clicked(bool check);
  void on_OM_z_U10_clicked(bool check);
  void on_OM_z_U20_clicked(bool check);
  void on_OM_z_U200_clicked(bool check);
  void on_OM_z_U2000_clicked(bool check);
  void on_OM_z_D1_clicked(bool check);
  void on_OM_z_D10_clicked(bool check);
  void on_OM_z_D20_clicked(bool check);
  void on_OM_z_D200_clicked(bool check);
  void on_OM_z_D2000_clicked(bool check);
  void on_OM_z_AddrFlake_clicked(bool check);
  void on_OM_z_Stamp_clicked(bool check);

  void on_Light_On_Button_clicked(bool check);
  void on_Light_Off_Button_clicked(bool check);
  void on_Light_Inten_valueChanged();

  void on_Sample_XY_Spd_valueChanged();
  void on_Sample_Z_Spd_valueChanged();
  void on_Sample_Theta_Spd_valueChanged();
  void on_OM_XY_Spd_valueChanged();
  void on_Sample_XY_Home_clicked(bool check);
  void on_OM_XY_Home_clicked(bool check);

  void on_Stamp_Taihi_clicked(bool check);
  void on_Stamp_Fukki_clicked(bool check);

  void on_Alignment_Pos_clicked(bool check);
  void on_Exchange_Pos_clicked(bool check);

 private:
  int init_argc;
  char **init_argv;
  ros::Publisher chatter_publisher;
  ros::Subscriber stamper_currtemp_subscriber;
  ros::Subscriber stamper_currtemp_sv_subscriber;
  ros::Subscriber stamper_currload_subscriber;
  ros::Subscriber stamper_keyence_subscriber;
  ros::Subscriber stamper_sample_xy_subscriber;
  ros::Subscriber stamper_om_xy_subscriber;
  ros::Publisher keyence_om_fwd_publisher;
  ros::Publisher keyence_om_back_publisher;
  ros::Publisher keyence_mask_fwd_publisher;
  ros::Publisher keyence_mask_back_publisher;
  ros::Publisher keyence_sample_cool_on_publisher;
  ros::Publisher keyence_sample_cool_off_publisher;
  ros::Publisher keyence_sample_blow_on_publisher;
  ros::Publisher keyence_sample_blow_off_publisher;
  ros::Publisher keyence_stage_lock_on_publisher;
  ros::Publisher keyence_stage_lock_off_publisher;
  ros::Publisher keyence_sample_vacuum_on_publisher;
  ros::Publisher keyence_sample_vacuum_off_publisher;
  ros::Publisher keyence_mask_chuck_on_publisher;
  ros::Publisher keyence_mask_chuck_off_publisher;
  ros::Publisher stamper_loadcell_zero_publisher;
  ros::Publisher stamper_temperature_set_publisher;

  ros::ServiceClient keyence_status_client;

  ros::Publisher stamper_nic_100a_cmd_on;
  ros::Publisher stamper_nic_100a_cmd_off;
  ros::Publisher stamper_nic_100a_cmd_inten;

  ros::Publisher stamper_om_xy_cmd_jog;
  ros::Publisher stamper_om_xy_cmd_stop;
  ros::Publisher stamper_om_xy_cmd_abs;
  ros::Publisher stamper_om_xy_cmd_vel;
  ros::Publisher stamper_om_xy_cmd_stp;
  ros::ServiceClient stamper_om_xy_get_currrent_pos_client;

  ros::Publisher stamper_sample_xy_cmd_jog;
  ros::Publisher stamper_sample_xy_cmd_abs;
  ros::Publisher stamper_sample_xy_cmd_stop;
  ros::Publisher stamper_sample_xy_cmd_vel;
  ros::Publisher stamper_sample_xy_cmd_stp;
  ros::ServiceClient stamper_sample_xy_get_current_pos_client;

  ros::Publisher stamper_sample_z_cmd_jog;
  ros::Publisher stamper_sample_z_cmd_abs;
  ros::Publisher stamper_sample_z_cmd_stop;
  ros::Publisher stamper_sample_z_cmd_vel;
  ros::Publisher stamper_sample_z_cmd_home;
  ros::Publisher stamper_sample_z_cmd_stp;
  ros::ServiceClient stamper_sample_z_wait_for_stop_client;
  ros::ServiceClient stamper_sample_z_get_current_pos_client;

  ros::Publisher stamper_sample_theta_cmd_jog;
  ros::Publisher stamper_sample_theta_cmd_abs;
  ros::Publisher stamper_sample_theta_cmd_stop;
  ros::Publisher stamper_sample_theta_cmd_vel;

  ros::Publisher stamper_nikon_cmd_revolv_next;
  ros::Publisher stamper_nikon_cmd_revolv_prev;
  ros::Publisher stamper_nikon_cmd_revolv_set;
  ros::Publisher stamper_nikon_cmd_z_u1;
  ros::Publisher stamper_nikon_cmd_z_u10;
  ros::Publisher stamper_nikon_cmd_z_u20;
  ros::Publisher stamper_nikon_cmd_z_u200;
  ros::Publisher stamper_nikon_cmd_z_u2000;
  ros::Publisher stamper_nikon_cmd_z_d1;
  ros::Publisher stamper_nikon_cmd_z_d10;
  ros::Publisher stamper_nikon_cmd_z_d20;
  ros::Publisher stamper_nikon_cmd_z_d200;
  ros::Publisher stamper_nikon_cmd_z_d2000;
  ros::Publisher stamper_nikon_cmd_z_abs;

  ros::Publisher stamper_sample_xy_cmd_home;
  ros::Publisher stamper_om_xy_cmd_home;

  QStringListModel logging_model;
};

}  // namespace tdmms_stamper_teleop

#endif /* tdmms_stamper_teleop_QNODE_HPP_ */
