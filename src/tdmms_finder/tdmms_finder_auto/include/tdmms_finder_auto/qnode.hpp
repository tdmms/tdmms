#ifndef tdmms_finder_auto_QNODE_HPP_
#define tdmms_finder_auto_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QThread>
#include <QStringListModel>
#include <QStandardItemModel>
#include <QTableWidgetItem>
#include <geometry_msgs/Point.h>
#include <QSpinBox>
#include <QComboBox>
#include <QFileInfoList>
#include <QTextBrowser>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <sensor_msgs/Joy.h>
#include <QCheckBox>
#include <QDateEdit>
#include <QtSql/QtSql>
#include <list>
#include "HalconCpp.h"
#include "HDevThread.h"
#include <ros/ros.h>
#include <string>
#include "../../../tdmms_finder_libdecision/include/tdmms_finder_hinterface.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace HalconCpp;
namespace tdmms_finder_auto {

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
  QTableWidget *tableWidget;

  enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };
  QTableWidget *SearchAreaTableWidget;
  QSpinBox *spinBox_Delay;
  QSpinBox *spinBox_StepX;
  QSpinBox *spinBox_StepY;
  QSpinBox *spinBox_Exposure;
  QComboBox *FinderLibrarycomboBox;
  QLabel *label_comment;
  QToolButton *Button_HaltFind;
  QSpinBox *spinBox_pic_count;
  QLineEdit *lineEdit_picFolder;
  QCheckBox *checkBox_SaveAll;
  QCheckBox *checkBox_simplemode;
  QSpinBox *spinBox_ChiptrayID;
  QSpinBox *spinBox_NumOfRows;
  QDoubleSpinBox *doubleSpinBox_cornerWeight;
  QComboBox *comboBox_cornerMask;
  QComboBox *comboBox_Crystal;
  QComboBox *comboBox_Exfoliator;
  QComboBox *comboBox_ExfoliatedPlace;
  QComboBox *comboBox_Wafer;
  QDateEdit *dateEdit_Exfoliated;
  QSpinBox *spinBox_ChiptrayID_DataManip;
  QSpinBox *spinBox_NumOfChips;
  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);
  void currpos_Callback(const geometry_msgs::Point &pnt);
  void joy_Callback(const sensor_msgs::Joy &joy_msg);

Q_SIGNALS:
  void loggingUpdated();
  void areaUpdated();
  void rosShutdown();

 public
Q_SLOTS:
  void on_Button_AddRecord_clicked();
  void on_Button_Sequence_clicked();
  void on_Button_StartFind_clicked();
  void on_Button_Add_clicked();
  void on_Button_AutoSearchArea_clicked();
  void on_Button_ClearAll_clicked();
  void on_Button_Delete_clicked();
  void on_Button_HaltFind_clicked();
  void on_Button_ResumeFind_clicked();
  void on_Button_SetFolder_clicked();
  void on_Button_SetP1_clicked();
  void on_Button_SetP2_clicked();
  void on_Button_SetP3_clicked();
  void on_Button_SetP4_clicked();
  void on_Exposure_SpinBox_valueChanged();
  void on_FinderLibrarycomboBox_currentIndexChanged(int i);
  void on_CheckBox_testmode_stateChanged(int state);
  void on_Button_SetPicFolder_clicked();
  void on_Button_SetFolderAll_clicked();
  void on_Button_AutoSearchImage_clicked();
  void on_Button_CaptureOrigin_clicked();
  void on_Button_CaptureAlignmentImage_clicked();

 private:
  int init_argc;
  char **init_argv;
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

  QSqlDatabase db;

  geometry_msgs::Point currpoint;
  geometry_msgs::Point startpoint;

  QStringListModel logging_model;

  HObject ho_Image;
  HTuple hv_AcqName;
  HTuple hv_Revision;
  HTuple hv_exposure;
  HTuple hv_AcqHandle;
  HTuple hv_Width;
  HTuple hv_Height;
  HTuple hv_WindowID;
  HTuple hv_WindowID_buf;
  void ExtractPointListFromTableWidget(
      std::list<geometry_msgs::Point> *PointList, int row, int stepx,
      int stepy);
  void SetObjectiveLensTartlet(int TartletPos);
  int SQLGetChipID(int chiptray_id, int pos_in_chiptray);
  int SQLGetMaximumSearchID(int id_chip);
  int SQLGetMaximumOriginID();
  int SQLGetMaximumChipID();
  void SQLInsertChip(int nofchips, int id_chiptray, int id_wafer,
                     QDateTime datetime, QString exfoliated_place,
                     int id_crystal, int id_exfoliator);
  void SQLInsertAlignmentImage(char *filename, int pos_alignment_x,
                               int pos_alignment_y, int id_search, int id_chip);
  void SQLSetDatetimeSearchStarted(int id_search, int id_chip);
  void SQLSetDatetimeSearchFinished(int id_search, int id_chip);
  bool SQLCheckDuplicate(int id_search, int id_chip, int point_x, int point_y, int thresh);
  void SQLInsertMetaImage(int flakecount, char *filename, int pos_x, int pos_y,
                          int id_search, int id_chip);
  void SQLInsertFlakeImage(int flakecount, int id_objective_lens,
                           char *filename, int point_x, int point_y,
                           int id_search, int id_chip, ObjectFeatures feature);
  void SQLInsertOriginImage(int id_origin_image, int id_search,
                            int id_search_chip, int pos_x, int pos_y,
                            char *filename);
  void SQLInsertSearch(int id_search, int id_chip, const char *search_program,
                       int target_layer_num_min, int target_layer_num_max);
  QStringList SQLGetExfoliatorName();
  QStringList SQLGetWaferNames();
  QStringList SQLGetCrystalNames();

  void ExecuteAutofocus();
  bool isInside(geometry_msgs::Point rect[4],
                geometry_msgs::Point pntinvestigate);
  void OpenSQLConnection(void);

  bool searching;
  enum E_currentState {
    Live,
    unLive,
    Check,
    Search,
    Halt
  };
  E_currentState currentState;

  typedef Herror (*tdmms_finder_Initialize_ptr)();
  typedef Herror (*tdmms_finder_Live_ptr)();
  typedef Herror (*tdmms_finder_CloseWindow_ptr)();
  typedef Herror (*tdmms_finder_OpenWindow_ptr)();
  typedef Herror (*tdmms_finder_Find_ptr)(bool *, int *, int *, unsigned int *);
  typedef Herror (*tdmms_finder_SetParameter_ptr)(char *, Hcpar *);
  typedef Herror (*tdmms_finder_GetParameter_ptr)(char *, Hcpar *);
  typedef Herror (*tdmms_finder_hello_ptr)();
  typedef Herror (*tdmms_finder_SaveImage_ptr)(char *);
  typedef Herror (*tdmms_finder_SaveImage_raw_ptr)(char *);
  typedef Herror (*tdmms_finder_SaveImage_affine_ptr)(char *);
  typedef Herror (*tdmms_finder_SaveRegion_ptr)(char *);
  typedef Herror (*tdmms_finder_ExtractFeatures_ptr)(ObjectFeatures *);

  tdmms_finder_hello_ptr fp_tdmms_finder_hello;
  tdmms_finder_Initialize_ptr fp_tdmms_finder_Initialize;
  tdmms_finder_Live_ptr fp_tdmms_finder_Live;
  tdmms_finder_CloseWindow_ptr fp_tdmms_finder_CloseWindow;
  tdmms_finder_OpenWindow_ptr fp_tdmms_finder_OpenWindow;
  tdmms_finder_Find_ptr fp_tdmms_finder_Find;
  tdmms_finder_SetParameter_ptr fp_tdmms_finder_SetParameter;
  tdmms_finder_GetParameter_ptr fp_tdmms_finder_GetParameter;
  tdmms_finder_SaveImage_ptr fp_tdmms_finder_SaveImage;
  tdmms_finder_SaveImage_raw_ptr fp_tdmms_finder_SaveImage_raw;
  tdmms_finder_SaveImage_affine_ptr fp_tdmms_finder_SaveImage_affine;
  tdmms_finder_SaveRegion_ptr fp_tdmms_finder_SaveRegion;
  tdmms_finder_ExtractFeatures_ptr fp_tdmms_finder_ExtractFeatures;

  void *dlhandle;
  QFileInfoList listLibraryFile;
  void changeLibrary();
  int yokoidou;
  int tateidou;
  int syasinnnokazu_yoko;
  int syasinnnokazu_tate;
  QString strDefaultImageFolder;
};

}  // namespace tdmms_finder_auto

#endif /* tdmms_finder_auto_QNODE_HPP_ */
