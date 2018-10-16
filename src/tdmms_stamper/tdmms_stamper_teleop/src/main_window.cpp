//  Copyright 2016.4 by Satoru Masubuchi
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/tdmms_stamper_teleop/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace tdmms_stamper_teleop {
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
                   SLOT(aboutQt()));
  ReadSettings();
  setWindowIcon(QIcon(":/images/1462281764_110.png"));
  ui.tab_manager->setCurrentIndex(0);
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));
  QObject::connect(ui.pushButton_OM_Forward, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_Forward_Button_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Back, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_Back_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Mask_Forward, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Mask_Forward_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Mask_Back, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Mask_Back_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_Cool_On, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Sample_Cool_On_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_Blow_On, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Sample_Blow_On_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_Vacuum_On, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_Vacuum_On_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Stage_Lock_On, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Stage_Lock_On_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Mask_Chuck_On, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Mask_Chuck_On_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Loadcell_ZeroSet, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Loadcell_ZeroSet_Button_clicked(bool)));

  QObject::connect(ui.spinBox_Temperature, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Temperature_valueChanged()));
  QObject::connect(ui.spinBox_OM_Z_Spd, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_OM_Z_Spd_valueChanged()));
  QObject::connect(ui.spinBox_Sample_Z_Spd, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Sample_Z_Spd_valueChanged()));
  QObject::connect(ui.spinBox_Sample_Theta_Spd, SIGNAL(valueChanged(int)),
                   &qnode, SLOT(on_Sample_Theta_Spd_valueChanged()));
  QObject::connect(ui.spinBox_Sample_XY_Spd, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Sample_XY_Spd_valueChanged()));
  QObject::connect(ui.spinBox_OM_XY_Spd, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_OM_XY_Spd_valueChanged()));

  QObject::connect(ui.pushButton_Light_On, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Light_On_Button_clicked(bool)));
  QObject::connect(ui.pushButton_Light_Off, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Light_Off_Button_clicked(bool)));
  QObject::connect(ui.spinBox_Light_Inten, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Light_Inten_valueChanged()));

  QObject::connect(ui.pushButton_Sample_XY_X_Minus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_XY_X_Minus_pressed()));
  QObject::connect(ui.pushButton_Sample_XY_X_Minus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_XY_X_Minus_released()));
  QObject::connect(ui.pushButton_Sample_XY_X_Plus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_XY_X_Plus_pressed()));
  QObject::connect(ui.pushButton_Sample_XY_X_Plus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_XY_X_Plus_released()));
  QObject::connect(ui.pushButton_Sample_XY_Y_Minus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_XY_Y_Minus_pressed()));
  QObject::connect(ui.pushButton_Sample_XY_Y_Minus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_XY_Y_Minus_released()));
  QObject::connect(ui.pushButton_Sample_XY_Y_Plus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_XY_Y_Plus_pressed()));
  QObject::connect(ui.pushButton_Sample_XY_Y_Plus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_XY_Y_Plus_released()));
  QObject::connect(ui.pushButton_Sample_XY_Center, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_XY_Center_clicked(bool)));

  QObject::connect(ui.pushButton_Sample_Z_Minus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_Z_Minus_pressed()));
  QObject::connect(ui.pushButton_Sample_Z_Minus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_Z_Minus_released()));
  QObject::connect(ui.pushButton_Sample_Z_Plus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_Z_Plus_pressed()));
  QObject::connect(ui.pushButton_Sample_Z_Plus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_Z_Plus_released()));
  QObject::connect(ui.pushButton_Sample_Z_Home, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Sample_Z_Home_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_Z_Point0, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Sample_Z_Point0_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_Z_Point1, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Sample_Z_Point1_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_Z_Point2, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Sample_Z_Point2_clicked(bool)));

  QObject::connect(ui.pushButton_Sample_Theta_Minus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_Theta_Minus_pressed()));
  QObject::connect(ui.pushButton_Sample_Theta_Minus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_Theta_Minus_released()));
  QObject::connect(ui.pushButton_Sample_Theta_Plus, SIGNAL(pressed()), &qnode,
                   SLOT(on_Sample_Theta_Plus_pressed()));
  QObject::connect(ui.pushButton_Sample_Theta_Plus, SIGNAL(released()), &qnode,
                   SLOT(on_Sample_Theta_Plus_released()));
  QObject::connect(ui.pushButton_Sample_Theta_Center, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_Theta_Center_clicked(bool)));

  QObject::connect(ui.pushButton_OM_XY_X_Minus, SIGNAL(pressed()), &qnode,
                   SLOT(on_OM_XY_X_Minus_pressed()));
  QObject::connect(ui.pushButton_OM_XY_X_Minus, SIGNAL(released()), &qnode,
                   SLOT(on_OM_XY_X_Minus_released()));
  QObject::connect(ui.pushButton_OM_XY_X_Plus, SIGNAL(pressed()), &qnode,
                   SLOT(on_OM_XY_X_Plus_pressed()));
  QObject::connect(ui.pushButton_OM_XY_X_Plus, SIGNAL(released()), &qnode,
                   SLOT(on_OM_XY_X_Plus_released()));

  QObject::connect(ui.pushButton_OM_XY_Y_Minus, SIGNAL(pressed()), &qnode,
                   SLOT(on_OM_XY_Y_Minus_pressed()));
  QObject::connect(ui.pushButton_OM_XY_Y_Minus, SIGNAL(released()), &qnode,
                   SLOT(on_OM_XY_Y_Minus_released()));
  QObject::connect(ui.pushButton_OM_XY_Y_Plus, SIGNAL(pressed()), &qnode,
                   SLOT(on_OM_XY_Y_Plus_pressed()));
  QObject::connect(ui.pushButton_OM_XY_Y_Plus, SIGNAL(released()), &qnode,
                   SLOT(on_OM_XY_Y_Plus_released()));
  QObject::connect(ui.pushButton_OM_XY_Center, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_XY_Center_clicked(bool)));
  QObject::connect(ui.pushButton_OM_XY_Home, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_XY_Home_clicked(bool)));

  QObject::connect(ui.pushButton_OM_Revolv_Next, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_Revolv_Next_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Revolv_Prev, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_Revolv_Prev_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_U1, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_U1_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_U10, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_U10_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_U20, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_U20_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_U200, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_U200_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_U2000, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_U2000_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_D1, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_D1_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_D10, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_D10_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_D20, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_D20_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_D200, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_D200_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_D2000, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_D2000_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_Stamp, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_Stamp_clicked(bool)));
  QObject::connect(ui.pushButton_OM_Z_AddrFlake, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_OM_z_AddrFlake_clicked(bool)));

  QObject::connect(ui.pushButton_OM_XY_X_Plus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_OM_XY_X_Plus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_OM_XY_X_Minus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_OM_XY_X_Minus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_OM_XY_Y_Plus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_OM_XY_Y_Plus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_OM_XY_Y_Minus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_OM_XY_Y_Minus_Stp_clicked(bool)));

  QObject::connect(ui.pushButton_Sample_XY_X_Plus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_XY_X_Plus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_XY_X_Minus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_XY_X_Minus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_XY_Y_Plus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_XY_Y_Plus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_XY_Y_Minus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_XY_Y_Minus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_XY_Home, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Sample_XY_Home_clicked(bool)));

  QObject::connect(ui.pushButton_Sample_Z_Plus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_Z_Plus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_Sample_Z_Minus_STP, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_Sample_Z_Minus_Stp_clicked(bool)));
  QObject::connect(ui.pushButton_Stamp_Taihi, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Stamp_Taihi_clicked(bool)));
  QObject::connect(ui.pushButton_Stamp_Fukki, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Stamp_Fukki_clicked(bool)));
  QObject::connect(ui.pushButton_Alignment_Pos, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Alignment_Pos_clicked(bool)));
  QObject::connect(ui.pushButton_Exchange_Pos, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Exchange_Pos_clicked(bool)));

  qnode.spinBox_Temperature = ui.spinBox_Temperature;
  qnode.lcdNumber_Temperature = ui.lcdNumber_Temperature;
  qnode.lcdNumber_Temperature_sv = ui.lcdNumber_Temperature_sv;
  qnode.lcdNumber_Load = ui.lcdNumber_Loadcell;
  qnode.lcdNumber_Sample_X = ui.lcdNumber_Sample_X;
  qnode.lcdNumber_Sample_Y = ui.lcdNumber_Sample_Y;
  qnode.lcdNumber_OM_X = ui.lcdNumber_OM_X;
  qnode.lcdNumber_OM_Y = ui.lcdNumber_OM_Y;
  qnode.spinBox_Light_Inten = ui.spinBox_Light_Inten;
  qnode.spinBox_Sample_Theta_Spd = ui.spinBox_Sample_Theta_Spd;
  qnode.spinBox_Sample_Z_Spd = ui.spinBox_Sample_Z_Spd;
  qnode.spinBox_Sample_XY_Spd = ui.spinBox_Sample_XY_Spd;
  qnode.spinBox_OM_Z_Spd = ui.spinBox_OM_Z_Spd;
  qnode.spinBox_OM_XY_Spd = ui.spinBox_OM_XY_Spd;
  qnode.spinBox_Sample_Z_STP = ui.spinBox_Sample_Z_STP;
  qnode.spinBox_Sample_XY_STP = ui.spinBox_Sample_XY_STP;
  qnode.spinBox_OM_XY_STP = ui.spinBox_Sample_XY_STP;

  qnode.pushButton_OM_Back = ui.pushButton_OM_Back;
  qnode.pushButton_OM_Forward = ui.pushButton_OM_Forward;
  qnode.pushButton_Mask_Back = ui.pushButton_Mask_Back;
  qnode.pushButton_Mask_Forward = ui.pushButton_Mask_Forward;
  qnode.pusuButton_ESD_Back = ui.pushButton_ESD_Back;
  qnode.pushButton_ESD_Forward = ui.pushButton_ESD_Forward;

  qnode.pushButton_Sample_Blow_On = ui.pushButton_Sample_Blow_On;
  qnode.pushButton_Sample_Cool_On = ui.pushButton_Sample_Cool_On;
  qnode.pushButton_Sample_Vacuum_On = ui.pushButton_Sample_Vacuum_On;
  qnode.pushButton_Mask_Chuck_On = ui.pushButton_Mask_Chuck_On;
  qnode.pushButton_Stage_Lock_On = ui.pushButton_Stage_Lock_On;

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

void MainWindow::updateLoggingView() { ui.view_logging->scrollToBottom(); }

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
  QSettings settings("Qt-Ros Package", "tdmms_stamper_teleop");
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
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_stamper_teleop");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  // settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace tdmms_stamper_teleop
