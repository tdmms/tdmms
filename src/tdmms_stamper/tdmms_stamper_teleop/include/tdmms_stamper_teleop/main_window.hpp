// Copyright 2016.4 by Satoru Masubuchi

#ifndef tdmms_stamper_teleop_MAIN_WINDOW_H
#define tdmms_stamper_teleop_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace tdmms_stamper_teleop {
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();   // Load up qt program settings at startup
  void WriteSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check);
  void on_checkbox_use_environment_stateChanged(int state);

  void updateLoggingView();  // no idea why this can't connect automatically

 private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace tdmms_stamper_teleop

#endif  // tdmms_stamper_teleop_MAIN_WINDOW_H
