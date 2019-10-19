/**
 * @file /include/tdmms_finder_dl/main_window.hpp
 *
 * @brief Qt based gui for tdmms_finder_dl.
 *
 * @date November 2010
 **/
#ifndef tdmms_finder_dl_MAIN_WINDOW_H
#define tdmms_finder_dl_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace tdmms_finder_dl {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
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
  void on_horizontalSlider_Exposure_RB_valueChanged(int value);
  void on_button_DeleteParams_RB_clicked();
  void on_button_AddParams_RB_clicked();
  void on_Button_LoadSettings_clicked();
  void on_Button_SaveSettings_clicked();
  void on_horizontalSlider_CentH_RB_valueChanged(int value);
  void on_horizontalSlider_DeltaH_RB_valueChanged(int value);
  void on_horizontalSlider_CentS_RB_valueChanged(int value);
  void on_horizontalSlider_DeltaS_RB_valueChanged(int value);
  void on_horizontalSlider_CentV_RB_valueChanged(int value);
  void on_horizontalSlider_DeltaV_RB_valueChanged(int value);
  void on_horizontalSlider_EdgeLow_RB_valueChanged(int value);
  void on_horizontalSlider_EdgeHigh_RB_valueChanged(int value);
  void on_horizontalSlider_EdgeAlpha_RB_valueChanged(int value);
  void on_horizontalSlider_EntropyMin_RB_valueChanged(int value);
  void on_horizontalSlider_EntropyMax_RB_valueChanged(int value);
  void on_horizontalSlider_AreaThresh_RB_valueChanged(int value);
  void on_horizontalSlider_HoleThresh_RB_valueChanged(int value);
  void on_horizontalSlider_CloseEdgeMinAmp_RB_valueChanged(int value);
  void on_horizontalSlider_CloseEdgeMaxGap_RB_valueChanged(int value);
  void on_horizontalSlider_Dilation_RB_valueChanged(int value);
  void on_horizontalSlider_Erosion_RB_valueChanged(int value);
  void on_horizontalSlider_Erosion2_RB_valueChanged(int value);
  void on_horizontalSlider_Dilation2_RB_valueChanged(int value);
  void on_horizontalSlider_EdgeDetect_RB_valueChanged(int value);

  void selectionChanged_DetectionParams();

 private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  QMap<QString, int> map_itemColIndx;
};

}  // namespace tdmms_finder_dl

#endif  // tdmms_finder_dl_MAIN_WINDOW_H
