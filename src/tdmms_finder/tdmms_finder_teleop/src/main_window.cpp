/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/tdmms_finder_teleop/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace tdmms_finder_teleop {

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);
  // Calling this incidentally connects all ui's triggers to on_...() callbacks
  // in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
                   SLOT(aboutQt()));
  // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/1463590669_141.png"));
  ui.tab_manager->setCurrentIndex(0);
  // ensure the first tab is showing - qt-designer should have this already
  // hardwired,
  // but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
   ** Logging
   **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));

  qnode.Light_Intensity_SpinBox = ui.Light_Intensity_SpinBox;
  qnode.Revolver_Pos_SpinBox = ui.Revolver_Pos_SpinBox;
  qnode.Pos_X_SpinBox = ui.Pos_X_SpinBox;
  qnode.Pos_Y_SpinBox = ui.Pos_Y_SpinBox;
  qnode.Step_SpinBox = ui.Step_SpinBox;
  qnode.Vel_Low_SpinBox = ui.Vel_Low_SpinBox;
  qnode.Vel_High_SpinBox = ui.Vel_High_SpinBox;
  qnode.Accl_SpinBox = ui.Accl_SpinBox;

  qnode.radioButton_LeftTop = ui.radioButton_LeftTop;
  qnode.radioButton_CenterTop = ui.radioButton_CenterTop;
  qnode.radioButton_RightTop = ui.radioButton_RightTop;
  qnode.radioButton_LeftCenter = ui.radioButton_LeftCenter;
  qnode.radioButton_Center = ui.radioButton_Center;
  qnode.radioButton_RightCenter = ui.radioButton_RightCenter;
  qnode.radioButton_LeftBottom = ui.radioButton_LeftBottom;
  qnode.radioButton_CenterBottom = ui.radioButton_CenterBottom;
  qnode.radioButton_RightBottom = ui.radioButton_RightBottom;

  qnode.Filter_Pos_SpinBox = ui.Filter_Pos_SpinBox;
  // qnode.Exposure_SpinBox = ui.Exposure_SpinBox;

  QObject::connect(ui.Light_Intensity_SpinBox, SIGNAL(valueChanged(int)),
                   &qnode, SLOT(on_Light_Intensity_SpinBox_valueChanged()));
  QObject::connect(ui.Step_SpinBox, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Step_SpinBox_valueChanged()));
  // QObject::connect(ui.Jog_SpinBox, SIGNAL(valueChanged(int)),
  //                 &qnode, SLOT(on_Jog_SpinBox_valueChanged()));

  // QObject::connect(ui.Exposure_SpinBox, SIGNAL(valueChanged(int)),
  //                 &qnode, SLOT(on_Exposure_SpinBox_valueChanged()));
  // QObject::connect(ui.Exposure_SpinBox, SIGNAL(valueChanged(int)),
  //                 ui.Exposure_horizontalSlider, SLOT(setValue(int)));
  // QObject::connect(ui.Exposure_horizontalSlider, SIGNAL(valueChanged(int)),
  //                 ui.Exposure_SpinBox, SLOT(setValue(int)));

  QObject::connect(ui.Vel_Low_SpinBox, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Vel_SpinBox_valueChanged()));
  QObject::connect(ui.Vel_High_SpinBox, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Vel_SpinBox_valueChanged()));
  QObject::connect(ui.Accl_SpinBox, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Vel_SpinBox_valueChanged()));

  QObject::connect(ui.Light_Intensity_SpinBox, SIGNAL(valueChanged(int)),
                   ui.Light_Intensity_horizontalSlider, SLOT(setValue(int)));
  QObject::connect(ui.Light_Intensity_horizontalSlider,
                   SIGNAL(valueChanged(int)), ui.Light_Intensity_SpinBox,
                   SLOT(setValue(int)));
  QObject::connect(ui.Light_Intensity_SpinBox, SIGNAL(valueChanged(int)),
                   &qnode, SLOT(on_Light_Intensity_SpinBox_valueChanged()));

  QObject::connect(ui.Light_ON_pushButton, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Light_ON_Button_clicked(bool)));

  QObject::connect(ui.Light_OFF_pushButton, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Light_OFF_Button_clicked(bool)));
  
  QObject::connect(ui.pushButton_Move_01, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_01_clicked(bool)));
  QObject::connect(ui.pushButton_Move_02, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_02_clicked(bool)));
  QObject::connect(ui.pushButton_Move_03, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_03_clicked(bool)));
  QObject::connect(ui.pushButton_Move_04, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_04_clicked(bool)));
  QObject::connect(ui.pushButton_Move_05, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_05_clicked(bool)));
  QObject::connect(ui.pushButton_Move_06, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_06_clicked(bool)));
  QObject::connect(ui.pushButton_Move_07, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_07_clicked(bool)));
  QObject::connect(ui.pushButton_Move_08, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_08_clicked(bool)));
  QObject::connect(ui.pushButton_Move_09, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_09_clicked(bool)));
  QObject::connect(ui.pushButton_Move_10, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_10_clicked(bool)));
  QObject::connect(ui.pushButton_Move_11, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_11_clicked(bool)));
  QObject::connect(ui.pushButton_Move_12, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_12_clicked(bool)));
  QObject::connect(ui.pushButton_Move_13, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_13_clicked(bool)));
  QObject::connect(ui.pushButton_Move_14, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_14_clicked(bool)));
  QObject::connect(ui.pushButton_Move_15, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_15_clicked(bool)));
  QObject::connect(ui.pushButton_Move_16, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_16_clicked(bool)));
  QObject::connect(ui.pushButton_Move_17, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_17_clicked(bool)));
  QObject::connect(ui.pushButton_Move_18, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_18_clicked(bool)));
  QObject::connect(ui.pushButton_Move_19, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_19_clicked(bool)));
  QObject::connect(ui.pushButton_Move_20, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_20_clicked(bool)));
  QObject::connect(ui.pushButton_Move_21, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_21_clicked(bool)));
  QObject::connect(ui.pushButton_Move_22, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_22_clicked(bool)));
  QObject::connect(ui.pushButton_Move_23, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_23_clicked(bool)));
  QObject::connect(ui.pushButton_Move_24, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_24_clicked(bool)));
  QObject::connect(ui.pushButton_Move_25, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_25_clicked(bool)));
  QObject::connect(ui.pushButton_Move_26, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_26_clicked(bool)));
  QObject::connect(ui.pushButton_Move_27, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_27_clicked(bool)));
  QObject::connect(ui.pushButton_Move_28, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_28_clicked(bool)));
  QObject::connect(ui.pushButton_Move_29, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_29_clicked(bool)));
  QObject::connect(ui.pushButton_Move_30, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_30_clicked(bool)));
  QObject::connect(ui.pushButton_Move_31, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_31_clicked(bool)));
  QObject::connect(ui.pushButton_Move_32, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_32_clicked(bool)));
  QObject::connect(ui.pushButton_Move_33, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_33_clicked(bool)));
  QObject::connect(ui.pushButton_Move_34, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_34_clicked(bool)));
  QObject::connect(ui.pushButton_Move_35, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_35_clicked(bool)));
  QObject::connect(ui.pushButton_Move_36, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Move_36_clicked(bool)));
  QObject::connect(ui.pushButton_HP, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_HP_clicked(bool)));
  QObject::connect(ui.Intensity_Descend_pushButton, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Intensity_Descend_Button_clicked(bool)));
  /*********************
   ** Auto Start
   **********************/
  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check) {
  if (ui.checkbox_use_environment->isChecked()) {
    if (!qnode.init()) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if (!qnode.init(ui.line_edit_master->text().toStdString(),
                    ui.line_edit_host->text().toStdString())) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
      ui.line_edit_topic->setReadOnly(true);
    }
  }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if (state == 0) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
  // ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
  ROS_INFO("update");
}

void MainWindow::updateareaView() { ROS_INFO("update"); }
/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(
      this, tr("About ..."),
      tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin "
         "Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_finder_teleop");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url =
      settings.value("master_url", QString("http://192.168.1.2:11311/"))
          .toString();
  QString host_url =
      settings.value("host_url", QString("192.168.1.3")).toString();
  // QString topic_name = settings.value("topic_name",
  // QString("/chatter")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  // ui.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if (checked) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
    // ui.line_edit_topic->setEnabled(false);
  }
  int lightintensity = settings.value("LightIntensity", 0).toInt();
  ui.Light_Intensity_SpinBox->setValue(lightintensity);
  ui.Light_Intensity_horizontalSlider->setValue(lightintensity);
  
  int step = settings.value("Step", 50).toInt();
  ui.Step_SpinBox->setValue(step);
  
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_finder_teleop");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  // settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("LightIntensity", ui.Light_Intensity_SpinBox->value());
  settings.setValue("Step", ui.Step_SpinBox->value());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace tdmms_finder_teleop
