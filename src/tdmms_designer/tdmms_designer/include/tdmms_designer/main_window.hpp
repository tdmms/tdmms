/**
 * @file /include/tdmms_designer/main_window.hpp
 *
 * @brief Qt based gui for tdmms_designer.
 *
 * @date November 2010
 **/
#ifndef tdmms_designer_MAIN_WINDOW_H
#define tdmms_designer_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QFileInfoList>
#include <QtSql>
#include <QWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QtGui/QMainWindow>
#include "qnode.hpp"
#include "ui_main_window.h"

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
** Namespace
*****************************************************************************/
using namespace HalconCpp;
namespace tdmms_designer {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
p * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();   // Load up qt program settings at startup
  void WriteSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();

 public
Q_SLOTS:
  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check);
  void on_checkbox_use_environment_stateChanged(int state);

  /******************************************
   ** Manual connections
   *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void updateStackedImage();
  void reloadImage();
  void horizontalSlider_Xofs_valueChanged(int value);
  void horizontalSlider_Yofs_valueChanged(int value);
  void horizontalSlider_theta_valueChanged(int value);
  void horizontalSlider_alpha_valueChanged(int value);
  void onMoveUp_clicked(bool check);
  void onMoveDown_clicked(bool check);
  void onReloadImage_clicked(bool check);
  void onButtonAdd_clicked(bool check);
  void onButtonDelete_clicked(bool check);
  void onButtonSubmit_clicked(bool check);
  void onButtonSave_clicked(bool check);
  void onButtonLoad_clicked(bool check);
  void onButtonSubmittostamper_clicked(bool check);
  void onButtonEdgeFitting_clicked(bool check);
  void onButtonClearEdgeFitting_clicked(bool check);
  void onButtonCheckout_clicked(bool check);
  void onButtonUnCheckout_clicked(bool check);
  void onComboboxEdgefilter_currentindex_changed(int index);
  void selectionChangedSlot(const QItemSelection & /*newSelection*/,
                            const QItemSelection & /*oldSelection*/);
  void selectionChangedSlot_stack();
  void cellEnteredSlot(int row, int column);
  void dropEvent(QDropEvent* event);
  bool dropOn(int *row, int *col, int *topIndex);

  void move(bool up);
  QList<QTableWidgetItem*> takeRow(int row);
  void setRow(int row, const QList<QTableWidgetItem*>& rowItems);

 private:
  Ui::MainWindowDesign ui;
  QFileInfoList listGraphicsFile;
  QString currentDb;
  QSqlDatabase db;
  QString filename;

  QNode qnode;
  HTuple hv_Width, hv_Height, hv_DeviceIdentifiers, hv_DeviceName,
      hv_DeviceVendor;
  HTuple hv_DeviceHandle, hv_WindowHandle;
  HTuple hv_syukushaku_preview;
  HTuple hv_WindowHandlePreview, hv_Matrix1, hv_Matrix2, hv_Matrix3;
  HTuple hv_WindowHandleStacked;
  HTuple hv_WindowHandleDiff;

  HObject ho_PreviewImage;
  HObject ho_StackedImage;
  HObject ho_BufferImage;
  HObject ho_PreviewImage_trans;
  HObject ho_GrayImage;
  HObject ho_ImaAmp;
  HObject ho_ImaDir;

  HObject ho_image_bg_diff, ho_Image_added_diff, ho_ImageSelected_diff;
  HObject ho_ImageSelected_affine_diff, ho_Image_Added_scaled_diff;
  HObject ho_ImageSelected_scaled_diff;
  HObject ho_Images;
  HObject ho_Image;
  HObject ho_ImageSelected, ho_ImageSelected_scaled;
  HTuple hv_alpha, hv_rot, hv_xofs, hv_yofs;
  HObject ho_Image_added;
  HObject ho_ImageSelected_affine;
  HObject ho_Image_Added_scaled;
  HObject ho_image_bg;
  HObject ho_image_bg_rgb;
  HObject ho_Edges, ho_Edges_Added;

  HTuple hv_RowBegin_Tuple;
  HTuple hv_RowEnd_Tuple;
  HTuple hv_ColBegin_Tuple;
  HTuple hv_ColEnd_Tuple;
};

}  // namespace tdmms_designer

#endif  // tdmms_designer_MAIN_WINDOW_H
