// Copyright 2016 by S. Masubuchi
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/tdmms_finder_auto/main_window.hpp"
#include <QStandardItemModel>

namespace tdmms_finder_auto {
using namespace Qt;
MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
                   SLOT(aboutQt()));

  ReadSettings();
  setWindowIcon(QIcon(":/images/1463590549_07.png"));
  ui.tab_manager->setCurrentIndex(0);
  ui.view_logging->setModel(qnode.loggingModel());

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));
  //QObject::connect(&qnode, SIGNAL(areaUpdated()), this, SLOT(updateareaView()));
  QObject::connect(ui.Button_Add, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_Add_clicked()));
  QObject::connect(ui.Button_AutoSearchArea, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_AutoSearchArea_clicked()));
  QObject::connect(ui.Button_Sequence, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_Sequence_clicked()));
  QObject::connect(ui.Button_Delete, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_Delete_clicked()));
  QObject::connect(ui.Button_HaltFind, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_HaltFind_clicked()));
  QObject::connect(ui.Button_SetFolder, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SetFolder_clicked()));
  QObject::connect(ui.Button_SetP1, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SetP1_clicked()));
  QObject::connect(ui.Button_SetP2, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SetP2_clicked()));
  QObject::connect(ui.Button_SetP3, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SetP3_clicked()));
  QObject::connect(ui.Button_SetP4, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SetP4_clicked()));
  QObject::connect(ui.Button_StartFind, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_StartFind_clicked()));

  QObject::connect(ui.Exposure_SpinBox, SIGNAL(valueChanged(int)), &qnode,
                   SLOT(on_Exposure_SpinBox_valueChanged()));
  QObject::connect(ui.Exposure_SpinBox, SIGNAL(valueChanged(int)),
                   ui.Exposure_horizontalSlider, SLOT(setValue(int)));
  QObject::connect(ui.Exposure_horizontalSlider, SIGNAL(valueChanged(int)),
                   ui.Exposure_SpinBox, SLOT(setValue(int)));
  QObject::connect(ui.FinderLibrarycomboBox, SIGNAL(currentIndexChanged(int)),
                   &qnode,
                   SLOT(on_FinderLibrarycomboBox_currentIndexChanged(int)));
  QObject::connect(ui.checkBox_testmode, SIGNAL(stateChanged(int)), &qnode,
                   SLOT(on_CheckBox_testmode_stateChanged(int)));

  QObject::connect(ui.Button_SetPicFolder, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SetPicFolder_clicked()));
  QObject::connect(ui.Button_SetFolderAll, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SetFolderAll_clicked()));
  QObject::connect(ui.Button_AutoSearchImage, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_AutoSearchImage_clicked()));
  QObject::connect(ui.Button_CaptureOrigin, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_CaptureOrigin_clicked()));
  QObject::connect(ui.Button_CaptureAlignmentImage, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_CaptureAlignmentImage_clicked()));
  QObject::connect(ui.Button_AddRecord, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_AddRecord_clicked()));
  qnode.spinBox_Delay = ui.spinBox_Delay;
  qnode.spinBox_StepX = ui.spinBox_StepX;
  qnode.spinBox_StepY = ui.spinBox_StepY;
  qnode.spinBox_Exposure = ui.Exposure_SpinBox;
  qnode.FinderLibrarycomboBox = ui.FinderLibrarycomboBox;
  qnode.label_comment = ui.label_comment;
  qnode.Button_HaltFind = ui.Button_HaltFind;
  qnode.lineEdit_picFolder = ui.lineEdit_picFolder;
  qnode.checkBox_SaveAll = ui.checkBox_SaveAll;
  qnode.checkBox_simplemode = ui.checkBox_simplemode;
  qnode.spinBox_pic_count = ui.spinBox_pic_count;
  qnode.spinBox_ChiptrayID = ui.spinBox_ChiptrayID;
  qnode.spinBox_NumOfRows = ui.spinBox_NumOfRows;
  qnode.doubleSpinBox_cornerWeight = ui.doubleSpinBox_cornerWeight;
  qnode.comboBox_cornerMask = ui.comboBox_cornerMask;
  qnode.comboBox_Crystal = ui.comboBox_Crystal;
  qnode.comboBox_Exfoliator = ui.comboBox_Exfoliator;
  qnode.comboBox_ExfoliatedPlace = ui.comboBox_ExfoliatedPlace;
  qnode.comboBox_Wafer = ui.comboBox_Wafer;
  qnode.spinBox_ChiptrayID_DataManip = ui.spinBox_ChiptrayID_DataManip;
  qnode.dateEdit_Exfoliated = ui.dateEdit_Exfoliated;
  qnode.spinBox_ChiptrayID_DataManip = ui.spinBox_ChiptrayID_DataManip;
  qnode.spinBox_NumOfChips = ui.spinBox_NumOfChips;


  ui.FinderLibrarycomboBox->count();

  qnode.SearchAreaTableWidget = ui.SearchAreaTableWidget;

  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

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

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_finder_auto");
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

  double weight = settings.value("weight", 0.07).toDouble();
  ui.doubleSpinBox_cornerWeight->setValue(weight);

  int comboindex = settings.value("mask", 1).toInt();
  ui.comboBox_cornerMask->setCurrentIndex(comboindex);

  int trayId = settings.value("trayid", 0).toInt();
  ui.spinBox_ChiptrayID->setValue(trayId);
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_finder_auto");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  // settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("mask", QVariant(ui.comboBox_cornerMask->currentIndex()));
  settings.setValue("weight", QVariant(ui.doubleSpinBox_cornerWeight->value()));
  settings.setValue("trayid", QVariant(ui.spinBox_ChiptrayID->value()));
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace tdmms_finder_auto
