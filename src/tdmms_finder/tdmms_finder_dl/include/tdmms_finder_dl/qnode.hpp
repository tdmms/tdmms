/**
 * @file /include/tdmms_finder_dl/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tdmms_finder_dl_QNODE_HPP_
#define tdmms_finder_dl_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#include <QThread>
#include <QStringListModel>
#include <QCheckBox>
#include <QString>
#include <QtSql/QtSql>
#include <QSpinBox>
#include <QTableWidget>
#include <QLineEdit>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <adm2/GetCurrentPos.h>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>

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

using namespace HalconCpp;
namespace tdmms_finder_dl {

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
  void prepareBGImage(QString fname_bg);
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
  QCheckBox *checkBox_TestMode_DL;
  QCheckBox *checkBox_Enhance_DL;
  QCheckBox *checkBox_RecordMask_DL;
  QCheckBox *checkBox_TestMode_RB;
  QCheckBox *checkBox_Enhance_RB;
  
  QTableWidget *tableWidget_SearchArea_DL;
  QTableWidget *tableWidget_SearchArea_RB;
  QTableWidget *tableWidget_DetectionParams_RB;
  QSpinBox *spinBox_pic_count;
  QSpinBox *spinBox_StepX;
  QSpinBox *spinBox_StepY;
  QSpinBox *spinBox_Delay;
  QSpinBox *spinBox_AreaThresh_DL;
  QSpinBox *spinBox_ChiptrayID_RB;
  QSpinBox *spinBox_ChiptrayID_DL;
  QLineEdit *lineEdit_picFolder;
  QLineEdit *lineEdit_Folder_DL;
  QLineEdit *lineEdit_Folder_RB;
  QLineEdit *lineEdit_S3Bucket;
  QLineEdit *lineEdit_BGImage;
  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);
  void runDetection(void);
  void currpos_Callback(const geometry_msgs::Point &pnt);
  QMap<QString, int> *map_itemColIndx;

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
 public
Q_SLOTS:
  void on_Button_CaptureChiptray_DL_clicked();
  void on_Button_CaptureChiptray_RB_clicked();
  void on_Button_DetectSiChips_DL_clicked();
  void on_Button_DetectSiChips_RB_clicked();
  void on_Button_Add_DL_clicked();
  void on_Button_Add_RB_clicked();
  void on_Button_InsertSearchRecord_DL_clicked();
  void on_Button_InsertSearchRecord_RB_clicked();
  void on_Button_CaptureEdge_DL_clicked();
  void on_Button_CaptureEdge_RB_clicked();
  void on_Button_SaveImage_clicked();
  void on_Button_SaveBG_clicked();
  void on_Button_LoadBG_clicked();

  /////////////////////////////////////////////////
  /// Functions for captureing alignment images
  /////////////////////////////////////////////////
  void CaptureAlignmentImage(QTableWidget* tableWidget_SearchArea,
                             int id_chiptray,
                             QString S3_key);
  void on_Button_CaptureAlignmentImage_DL_clicked();
  void on_Button_CaptureAlignmentImage_RB_clicked();
  
  void on_Button_StartFind_DL_clicked();
  void on_Button_StartFind_RB_clicked();
  void on_Button_SetKey_DL_clicked();
  void on_Button_SetKey_RB_clicked();
  void on_Button_StartSequence_DL_clicked();
  void on_Button_StartSequence_RB_clicked();
  void on_Button_OpenTestWindows_RB_clicked();
  void on_Button_CloseTestWindows_RB_clicked();
  void on_Button_PrepareManualRegistration_RB_clicked();
  void on_Button_PrepareManualRegistration_DL_clicked();
  void on_Button_ManualRegistration_RB_clicked();
  void on_Button_ManualRegistration_DL_clicked();
 private:
  std::vector<std::string> split(std::string &input, char delimiter);
  void SQLInsertSearch(int id_search, int id_chip, const char *search_program,
                       int target_layer_num_min, int target_layer_num_max);
  int SQLGetChipID(int chiptray_id, int pos_in_chiptray);
  int SQLGetMaximumSearchID(int id_chip);
  void SQLInsertSearch(int id_search, int id_chip, int step_x, int step_y);
  int SQLGetNumOfChips(int id_chiptray);
  int SQLGetMaximumEdgeID();
  void SQLInsertEdgeImage(int idimage_edge, int id_search, int id_chip,
                          int id_lens, int pos_x, int pos_y, char *s3_bucket,
                          char *s3_key, const char *s3_url);
  void SQLInsertAlignmentImage(int id_search, int id_chip, int id_lens,
                               int pos_x, int pos_y, char *s3_bucket,
                               char *s3_key, const char *s3_url);
  int SQLInsertImage(int id_search, int id_chip, int id_lens, int pos_x,
                     int pos_y, char *s3_bucket, char *s3_key,
                     const char *s3_url, bool send_training, bool centralized, bool manual);
  bool SQLExecQuery(QSqlQuery *q);
  int SQLInsertObject(int id_image, int classid, int roi_x1, int roi_y1,
                      int roi_x2, int roi_y2, double score, int area,
                      int point_x, int point_y);
  bool SQLCheckDuplicate(int id_search, int id_chip,
                         int point_x, int point_y,
                         int thresh);
  QString SQLSelectMaterialName (int idchiptray);
  void runDLDetection(int id_image, char *filename, int point_x, int point_y, bool test_mode);
  void runRBDetection(int id_image, char* filename, int point_x, int point_y, bool test_mode, bool setting_mode);
  
  void SetObjectiveLensTartlet(int TartletPos);
  bool SaveImage_affine(char *filename);
  Herror SaveImage_raw(char *filename);
  bool SaveImage_jpg(char *filename);
  void ExtractPointListFromTableWidget(
      QTableWidget *tableWidget,
      std::list<geometry_msgs::Point> *PointList, int row, int stepx,
      int stepy);
  bool isInside(geometry_msgs::Point rect[4], geometry_msgs::Point pnt);
  void ExecuteAutofocus();
  void parseRBParameters();
  int recordCentralizedImages(int id_image, int area_thresh, int row, int id_search, int id_chip);
  int recordCentralizedImages_RB(int id_image, int area_thresh, int row, int id_search, int id_chip);
  void CaptureChiptray(int id_chiptray);
  int extractPosInChiptray(int pos_x, int pos_y);
  int init_argc;
  char **init_argv;
  QString temp_filename;
  std_srvs::Empty emp_srv;
  std_msgs::Empty emp_msg;

  enum E_currentState {
    Live,
    unLive,
    Check,
    Search,
    Halt,
    Freeze
  };
  E_currentState currentState;
  int flakecount;
  
  ros::Publisher chatter_publisher;
  ros::Subscriber cmd_currpos_subscriber;
  ros::Publisher adm2_abs_publisher;
  ros::Subscriber joy_subscriber;
  ros::Publisher afc_5_publisher;
  ros::Publisher afc_5_publisher_sc0;
  ros::Publisher adm2_stp_publisher;
  ros::Publisher lv_ncnt_n_publisher;
  ros::Publisher nic_100a_inten_publisher;
  ros::ServiceClient adm2_wait_for_stop_client;
  ros::Publisher adm2_vel_publisher;
  ros::Publisher finder_network_publisher;
  ros::ServiceClient finder_s3_upload_service;
  ros::ServiceClient adm2_get_current_pos_client;
  QStringListModel logging_model;

  int step_x, step_y;
  int numsteps_x, numsteps_y;
  geometry_msgs::Point origin;
  geometry_msgs::Point currpoint;
  QString strDefaultImageFolder;
  QString strDefaultTemporaryFolder;
  HTuple hv_DeviceIdentifiers;
  HTuple hv_DeviceName;
  HTuple hv_DeviceVendor;
  HTuple hv_DeviceHandle;
  HTuple hv_WindowHandleOM;
  HObject ho_OMImage;
  HTuple hv_windowScale;
  HTuple hv_Width;
  HTuple hv_Height;
  HTuple hv_homedir;
  HTuple hv_AcqName;
  HTuple hv_exposure;
  HTuple hv_AcqHandle;
  QSqlDatabase db;
  QSqlQuery *p_query;

  // Local iconic variables
  HObject ho_Image_BG, ho_Image_BG_R, ho_Image_BG_G;
  HObject ho_Image_BG_B, ho_Image_BG_H, ho_Image_BG_S, ho_Image_BG_V;
  HObject ho_Image, ho_ImageGauss, ho_Image_R, ho_Image_G;
  HObject ho_Image_B, ho_Image_H, ho_Image_S, ho_Image_V;
  HObject ho_ImageGrey, ho_ImaAmp, ho_ImaDir, ho_Region_Edge;
  HObject ho_connectedRegion_Edge, ho_RegionClosed, ho_edgesRegion;
  HObject ho_Image_Shifted, ho_Image_Conv;
  HObject ho_RegionLine_top, ho_RegionLine_bottom, ho_RegionLine_left;
  HObject ho_RegionLine_right, ho_RegionDifference, ho_ImageGrey_conv;
  HObject ho_ImageGrey_8, ho_RegionS, ho_ImageSub_H, ho_ImageSub_S;
  HObject ho_ImageSub_V, ho_Region_H, ho_Region_S, ho_Region_V;
  HObject ho_RegionI, ho_RegionIII, ho_SelectedRegions, ho_SelectedFlake;
  HObject ho_edgesRegion_Scaled, ho_RegionDifference_Scaled;
  HObject ho_Region_H_Scaled, ho_Region_S_Scaled, ho_Region_V_Scaled;
  HObject ho_SelectedRegions_scaled;
  HTuple hv_HomMat2DIdentity, hv_HomMat2DScale0125;
  HObject ho_GrayImage, ho_GrayImageAffinTrans;
  HObject ho_GrayImage_shift;

  // Local control variables
  HTuple hv_toolbar_offset;
  HTuple hv_WindowHandle_Edge, hv_WindowHandle_H, hv_WindowHandle_S;
  HTuple hv_WindowHandle_V, hv_WindowHandle_H_th, hv_WindowHandle_S_th;
  HTuple hv_WindowHandle_V_th, hv_WindowHandle_ClosedRegions;
  HTuple hv_WindowHandle_Result, hv_cent_H, hv_delta_H, hv_cent_S;
  HTuple hv_delta_S, hv_cent_V, hv_delta_V, hv_edge_low;
  HTuple hv_edge_high, hv_edge_alpha, hv_entropy_min, hv_entropy_max;
  HTuple hv_area_threshold, hv_area_holes_threshold, hv_close_edges_min_amp;
  HTuple hv_close_edges_max_gap, hv_p_conduct_erosion_dilation;
  HTuple hv_p_dilation, hv_p_erosion, hv_area, hv_Areas;
  HTuple hv_PointsY, hv_PointsX, hv_Number, hv_maxvalue;
  HTuple hv_index, hv_feature_values, hv_Rows, hv_Columns;

  HTuple hv_cent_Hs, hv_delta_Hs, hv_cent_Ss;
  HTuple hv_delta_Ss, hv_cent_Vs, hv_delta_Vs, hv_edge_lows;
  HTuple hv_edge_highs, hv_edge_alphas, hv_entropy_mins;
  HTuple hv_entropy_maxs, hv_area_thresholds, hv_area_holes_thresholds;
  HTuple hv_close_edges_min_amps, hv_close_edges_max_gaps, hv_p_dilations;
  HTuple hv_p_erosions;
  HTuple hv_p_dilation2s;
  HTuple hv_p_erosion2s;
  HTuple hv_p_dilation2;
  HTuple hv_p_erosion2;
  HTuple hv_p_conduct_edge_detections;
  HTuple hv_p_conduct_edge_detection;
  HTuple hv_lb_H, hv_ub_H;
  HTuple hv_lb_S, hv_ub_S, hv_lb_V, hv_ub_V;

  bool issettingMode;
};

}  // namespace tdmms_finder_dl

#endif /* tdmms_finder_dl_QNODE_HPP_ */
