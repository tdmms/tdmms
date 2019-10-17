/**
 * @file /include/tdmms_autostamp_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tdmms_autostamp_master_QNODE_HPP_
#define tdmms_autostamp_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QRadioButton>
#include <QSpinBox>
#include <QLineEdit>
#include <QTableWidget>
#include <QtSql>
#include <QString>
#include <QDoubleSpinBox>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tdmms_autostamp_flake_positioner_action/PositionFlakeAction.h>
#include <tdmms_autostamp_flake_positioner_action_fast/PositionFlakeFastAction.h>
#include <tdmms_autostamp_flake_positioner_action_fast2/PositionFlakeFast2Action.h>
#include <tdmms_autostamp_chip_transfer_action/TransferChipAction.h>
#include <tdmms_autostamp_flake_aligner_xy10x_action/AlignFlakeXY10xAction.h>
#include <tdmms_autostamp_flake_aligner_xy10x2_action/AlignFlakeXY10x2Action.h>
#include <tdmms_autostamp_flake_aligner_xy10x_fine_action/AlignFlakeXY10x_FineAction.h>
#include <tdmms_autostamp_flake_aligner_xytheta5x_action/AlignFlakeXYTheta5xAction.h>
#include <tdmms_autostamp_flake_aligner_xytheta5x2_action/AlignFlakeXYTheta5x2Action.h>
#include <tdmms_autostamp_flake_aligner_ncc_action/AlignFlakeNCCAction.h>
#include <tdmms_autostamp_stamp_action/StampAction.h>
#include <tdmms_autostamp_autofocus_action/AutoFocusAction.h>
#include <tdmms_autostamp_autofocus_action/AutoFocusResult.h>
#include <transfer_linear_yamaha/AbsMove.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_autostamp_master {

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
  void doneAutoFocusCb(
      const actionlib::SimpleClientGoalState &state,
      const tdmms_autostamp_autofocus_action::AutoFocusResultConstPtr &result);
  void activeAutoFocusCb();
  void feedbackAutoFocusCb(
      const tdmms_autostamp_autofocus_action::AutoFocusActionFeedbackConstPtr &
          feedback);
  bool setFileVer(int _fileVer);
  QSpinBox *spinBox_Auto_Stamp_Up_Speed;
  QSpinBox *spinBox_Auto_Stamp_Time;
  QSpinBox *spinBox_Auto_Stamp_Load;
  QSpinBox *spinBox_Auto_Stamp_Down_Speed;
  QSpinBox *spinBox_Auto_Stamp_OMatStamp;
  QSpinBox *spinBox_OriginIndex;
  QSpinBox *spinBox_Auto_Stamp_ZatStamp;
  QSpinBox *spinBox_PalletNo_1;
  QSpinBox *spinBox_PalletNo_2;
  QSpinBox *spinBox_PalletNo_3;
  QSpinBox *spinBox_ChiptrayNo_1;
  QSpinBox *spinBox_ChiptrayNo_2;
  QSpinBox *spinBox_ChiptrayNo_3;
  QLineEdit *lineEdit_Filename;
  QLineEdit *lineEdit_Xofs;
  QLineEdit *lineEdit_Yofs;
  QDoubleSpinBox *doubleSpinBox_Rotangle;
  QDoubleSpinBox *doubleSpinBox_LinearStage;
  QSpinBox *spinBox_PosinChiptray;
  QSpinBox *spinBox_ChiptrayNo;
  QTableWidget *tdmtableWidget;
  QSpinBox *spinBox_Align_Offset_X;
  QSpinBox *spinBox_Align_Offset_Y;
  QSpinBox *spinBox_Align_Offset_Theta_X;
  QSpinBox *spinBox_Align_Offset_Theta_Y;
  QDoubleSpinBox *doubleSpinBox_Align_Offset_Scale;
  QSpinBox *spinBox_AutoFocus_Count;
  QSpinBox *spinBox_AutoFocus_Step;
  QSpinBox *spinBox_AutoFocus_Zpos;
  QSpinBox *spinBox_AutoFocus_Start;
  QSpinBox *spinBox_ImageID;
  QSpinBox *spinBox_ChipID;
  QDoubleSpinBox *doubleSpinBox_Align_NCC_Scale_X;
  QDoubleSpinBox *doubleSpinBox_Align_NCC_Scale_Y;
  QLineEdit *lineEdit_picFolder;
  QSpinBox *spinBox_pic_count;
  
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
  void filename_updated_Callback(const std_msgs::String &fname);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

 public
Q_SLOTS:
  bool on_StartAutostamp_Full_clicked(bool check);
  bool on_StartAutostamp_clicked(bool check);
  void on_StartStep_clicked(bool check);
  void on_LoadTray_clicked(bool check);
  bool on_LoadChip_clicked(bool check);
  bool on_AddressFlake_clicked(bool check);
  bool on_AddressFlake2_clicked(bool check);
  bool on_ALignFlakeXY10x_clicked(bool check);
  bool on_AlignFlakeXY10x2_clicked(bool check);
  bool on_AlignFlakeXY10x_Fine_clicked(bool check);
  bool on_AlignFLakeTheta5x_clicked(bool check);
  bool on_AlignFlakeTheta5x2_clicked(bool check);
  bool on_AlignFlakeNCC_clicked(bool check);
  void on_AlignFlakeTheta10x_clicked(bool check);
  bool on_Stamp_clicked(bool check);
  bool on_UnloadChip_clicked(bool check);
  void on_UnloadTray_clicked(bool check);
  bool on_AutoAlign_clicked(bool check);
  bool on_AutoFocus_clicked(bool check);
  void on_LoadFile_clicked(bool check);
  void on_SaveFile_clicked(bool check);
  void on_Suspend_clicked(bool check);
  void on_Resume_clicked(bool check);
  void on_Stop_clicked(bool check);
  void on_MoveLinearStage_clicked(bool check);
  void on_SetZstage_clicked(bool check);
  void on_Stamp_Fukki_clicked(bool check);
  void on_Stamp_Taihi_clicked(bool check);
  bool on_Exchange_Pos_clicked(bool check);
  bool on_Alignment_Pos_clicked(bool check);
  bool on_Alignment_Pos_Low_clicked(bool check);
  bool on_AddressFlakeFast_clicked(bool check);
  bool on_CaptureImage(bool check);
  bool on_Skip_clicked(bool check);
  bool move_Alignment_Pos(int zpos);

 private:
  bool SQLSelectPositionXY(int *point_x_target, int *point_y_target,
                           QString filename);
  int SQLSelectSearchID(QString filename);
  QString SQLSelectFilenameFlakeImage(int position_x, int position_y,
                                      int search_id, int objective_lens);
  int SQLSelectSearchID2(unsigned long long id_image);
  bool SQLSelectPositionXY2(int *point_x_target, int *point_y_target,
                            unsigned long long id_image);
  int SQLSelectChipID2(unsigned long long id_image);
  QString SQLSelectFilenameFlakeImage2(int position_x, int position_y,
                                       int search_id, int chip_id, int objective_lens);
  int fileVer;
  int init_argc;
  int sample_z_stage_pos_w_alignment;
  char **init_argv;
  bool skip_requested;
  ros::Publisher chatter_publisher;
  QStringListModel logging_model;
  QString currentDb;
  QSqlDatabase db;
  QSqlDatabase db2;
  ros::Subscriber designer_filename_subscriber;
  ros::ServiceClient transfer_linear_cmd_abs_move;
  ros::Publisher stamper_sample_z_cmd_vel;
  ros::Publisher stamper_sample_z_cmd_abs;
  ros::ServiceClient stamper_sample_z_wait_for_stop_client;
  ros::ServiceClient stamper_sample_z_get_current_pos_client;
  ros::ServiceClient stamper_sample_xy_wait_for_stop_client;
  ros::ServiceClient stamper_sample_xy_get_current_pos_client;
  ros::ServiceClient stamper_om_xy_get_currrent_pos_client;
  ros::ServiceClient stamper_om_xy_wait_for_stop_client;
  ros::ServiceClient autostamp_camera_streamer_capture;
  ros::ServiceClient autostamp_camera_streamer_capture_timestamp;
  ros::Publisher stamper_sample_xy_cmd_abs;
  ros::Publisher stamper_nikon_cmd_z_abs;
  ros::Publisher keyence_sample_vacuum_on_publisher;
  ros::Publisher keyence_sample_vacuum_off_publisher;
  ros::Publisher keyence_om_fwd_publisher;
  ros::Publisher keyence_om_back_publisher;
  ros::Publisher keyence_mask_back_publisher;
  ros::Publisher keyence_mask_fwd_publisher;
  ros::Publisher stamper_sample_xy_cmd_vel;
  ros::Publisher stamper_sample_xy_cmd_jog;
  ros::Publisher stamper_sample_xy_cmd_stop;
  ros::Publisher stamper_sample_xy_cmd_stp;
  ros::Publisher stamper_sample_xy_cmd_home;
  ros::Publisher stamper_sample_theta_cmd_vel;
  ros::Publisher stamper_sample_theta_cmd_jog;
  ros::Publisher stamper_sample_theta_cmd_abs;
  ros::Publisher stamper_sample_theta_cmd_stop;
  ros::Publisher stamper_om_xy_cmd_vel;
  ros::Publisher stamper_om_xy_cmd_stp;

  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_positioner_action_fast::PositionFlakeFastAction>
      ac_flake_positioner_fast;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_positioner_action_fast2::PositionFlakeFast2Action>
      ac_flake_positioner_fast2;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_positioner_action::PositionFlakeAction>
      ac_flake_positioner;
  actionlib::SimpleActionClient<
      tdmms_autostamp_chip_transfer_action::TransferChipAction>
      ac_chip_transfer;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_aligner_xy10x_action::AlignFlakeXY10xAction>
      ac_flake_aligner_xy10x;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_aligner_xy10x2_action::AlignFlakeXY10x2Action>
      ac_flake_aligner_xy10x2;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_aligner_xytheta5x_action::AlignFlakeXYTheta5xAction>
      ac_flake_aligner_xytheta5x;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_aligner_xytheta5x2_action::AlignFlakeXYTheta5x2Action>
      ac_flake_aligner_xytheta5x2;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_aligner_xy10x_fine_action::
          AlignFlakeXY10x_FineAction> ac_flake_aligner_xy10x_fine;
  actionlib::SimpleActionClient<tdmms_autostamp_stamp_action::StampAction>
      ac_stamp;
  actionlib::SimpleActionClient<
      tdmms_autostamp_autofocus_action::AutoFocusAction> ac_autofocus;
  actionlib::SimpleActionClient<
      tdmms_autostamp_flake_aligner_ncc_action::AlignFlakeNCCAction>
      ac_flake_aligner_ncc;
};

}  // namespace tdmms_autostamp_master

#endif /* tdmms_autostamp_master_QNODE_HPP_ */
