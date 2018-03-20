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
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>

#include <QtGui>
#include <QMessageBox>
#include <QDir>
#include <iostream>
#include "../include/tdmms_autostamp_master/main_window.hpp"

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

namespace tdmms_autostamp_master {
using namespace Qt;
using namespace HalconCpp;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to
                     // on_...() callbacks in this class.
  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt()));  // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/1463591406_stack.png"));
  ui.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing -
                                       // qt-designer should have this already
                                       // hardwired, but often loses it
                                       // (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  QObject::connect(ui.pushButton_SetPicFolder, SIGNAL(clicked(bool)), this,
                   SLOT(on_Button_SetPicFolder_clicked(bool)));
  QObject::connect(ui.pushButton_StartAutostamp, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_StartAutostamp_clicked(bool)));
  QObject::connect(ui.pushButton_StartStep, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_StartStep_clicked(bool)));
  QObject::connect(ui.pushButton_LoadTray, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_LoadTray_clicked(bool)));
  QObject::connect(ui.pushButton_LoadChip, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_LoadChip_clicked(bool)));
  QObject::connect(ui.pushButton_AddressFlake, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_AddressFlake_clicked(bool)));
  QObject::connect(ui.pushButton_AddressFlakeFast, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_AddressFlakeFast_clicked(bool)));
  QObject::connect(ui.pushButton_AlignFlakeXY10x, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_ALignFlakeXY10x_clicked(bool)));
  QObject::connect(ui.pushButton_AlignFlakeXY10x_Fine, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_AlignFlakeXY10x_Fine_clicked(bool)));
  QObject::connect(ui.pushButton_AlignFlakeNCC, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_AlignFlakeNCC_clicked(bool)));
  QObject::connect(ui.pushButton_AlignFlakeTheta5x, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_AlignFLakeTheta5x_clicked(bool)));
  QObject::connect(ui.pushButton_AlignFlakeTheta10x, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_AlignFlakeTheta10x_clicked(bool)));
  QObject::connect(ui.pushButton_Stamp, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Stamp_clicked(bool)));
  QObject::connect(ui.pushButton_UnloadChip, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_UnloadChip_clicked(bool)));
  QObject::connect(ui.pushButton_AutoFocus, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_AutoFocus_clicked(bool)));
  QObject::connect(ui.pushButton_UnloadTray, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_UnloadTray_clicked(bool)));
  QObject::connect(ui.pushButton_AutoAlign, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_AutoAlign_clicked(bool)));
  QObject::connect(ui.pushButton_SaveFile, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_SaveFile_clicked(bool)));
  QObject::connect(ui.pushButton_Suspend, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Suspend_clicked(bool)));
  QObject::connect(ui.pushButton_Resume, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Resume_clicked(bool)));
  QObject::connect(ui.pushButton_Stop, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Stop_clicked(bool)));
  QObject::connect(ui.pushButton_MoveLinearStage, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_MoveLinearStage_clicked(bool)));
  QObject::connect(ui.pushButton_SetZstage, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_SetZstage_clicked(bool)));
  QObject::connect(ui.pushButton_Alignment_Pos, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Alignment_Pos_clicked(bool)));
  QObject::connect(ui.pushButton_Exchange_Pos, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Exchange_Pos_clicked(bool)));
  QObject::connect(ui.pushButton_Stamp_Fukki, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Stamp_Fukki_clicked(bool)));
  QObject::connect(ui.pushButton_Stamp_Taihi, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Stamp_Taihi_clicked(bool)));
  QObject::connect(ui.pushButton_StartAutostamp_Full, SIGNAL(clicked(bool)),
                   &qnode, SLOT(on_StartAutostamp_Full_clicked(bool)));
  QObject::connect(ui.pushButton_CaptureImage, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_CaptureImage(bool)));
  QObject::connect(ui.pushButton_Skip, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Skip_clicked(bool)));
  qnode.spinBox_Auto_Stamp_Up_Speed = ui.spinBox_Auto_Stamp_Up_Speed;
  qnode.spinBox_Auto_Stamp_Down_Speed = ui.spinBox_Auto_Stamp_Down_Speed;
  qnode.spinBox_Auto_Stamp_Load = ui.spinBox_Auto_Stamp_Load;
  qnode.spinBox_Auto_Stamp_Time = ui.spinBox_Auto_Stamp_Time;
  qnode.spinBox_Auto_Stamp_OMatStamp = ui.spinBox_Auto_Stamp_OMatStamp;
  qnode.spinBox_Auto_Stamp_ZatStamp = ui.spinBox_Auto_Stamp_ZatStamp;
  qnode.doubleSpinBox_LinearStage = ui.doubleSpinBox_LinearStage;
  qnode.lineEdit_Filename = ui.lineEdit_Filename;
  qnode.lineEdit_Xofs = ui.lineEdit_Xofs;
  qnode.lineEdit_Yofs = ui.lineEdit_Yofs;
  qnode.doubleSpinBox_Rotangle = ui.doubleSpinBox_Rotangle;
  qnode.spinBox_PosinChiptray = ui.spinBox_PosinChiptray;
  qnode.spinBox_ChiptrayNo = ui.spinBox_ChiptrayNo;
  qnode.spinBox_OriginIndex = ui.spinBox_OriginIndex;
  qnode.spinBox_PalletNo_1 = ui.spinBox_PalletNo_1;
  qnode.spinBox_PalletNo_2 = ui.spinBox_PalletNo_2;
  qnode.spinBox_PalletNo_3 = ui.spinBox_PalletNo_3;
  qnode.spinBox_ChiptrayNo_1 = ui.spinBox_ChiptrayNo_1;
  qnode.spinBox_ChiptrayNo_2 = ui.spinBox_ChiptrayNo_2;
  qnode.spinBox_ChiptrayNo_3 = ui.spinBox_ChiptrayNo_3;
  qnode.tdmtableWidget = ui.tdmtableWidget;
  qnode.spinBox_Align_Offset_Theta_X = ui.spinBox_Align_Offset_Theta_X;
  qnode.spinBox_Align_Offset_Theta_Y = ui.spinBox_Align_Offset_Theta_Y;
  qnode.spinBox_Align_Offset_X = ui.spinBox_Align_Offset_X;
  qnode.spinBox_Align_Offset_Y = ui.spinBox_Align_Offset_Y;
  qnode.doubleSpinBox_Align_Offset_Scale = ui.doubleSpinBox_Align_Offset_Scale;
  qnode.spinBox_AutoFocus_Step = ui.spinBox_Autofocus_Step;
  qnode.spinBox_AutoFocus_Count = ui.spinBox_Autofocus_Count;
  qnode.spinBox_AutoFocus_Zpos = ui.spinBox_Autofocus_Zpos;
  qnode.spinBox_AutoFocus_Start = ui.spinBox_Autofocus_Start;
  qnode.doubleSpinBox_Align_NCC_Scale_X = ui.doubleSpinBox_Align_NCC_Scale_X;
  qnode.doubleSpinBox_Align_NCC_Scale_Y = ui.doubleSpinBox_Align_NCC_Scale_Y;
  qnode.spinBox_pic_count = ui.spinBox_pic_count;
  qnode.lineEdit_picFolder = ui.lineEdit_picFolder;
  QObject::connect(ui.pushButton_LoadFile, SIGNAL(clicked(bool)), this,
                   SLOT(on_LoadFile_clicked(bool)));
  QObject::connect(ui.tdmtableWidget, SIGNAL(itemSelectionChanged()), this,
                   SLOT(selectionChangedSlot_stack()));
  ui.tdmtableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
  ui.tdmtableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);

  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));

  /*********************
  ** Auto Start
  **********************/
  ROS_INFO("Autostamp Master:connecting");
  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }
  QString ImageInit;
  ImageInit = QDir::homePath() +
      "/tdmms_data/images/01/o_0000000001_-000019992_0000014657_0000000000.jpg";
  ReadImage(&ho_ImagePreview, ImageInit.toLatin1().data());

  GetImageSize(ho_ImagePreview, &hv_Width, &hv_Height);
  //  Window Open
  SetWindowAttr("window_title", "Preview Image");
  OpenWindow(480 * 1.5 + 100, 0, hv_Width * 0.5, hv_Height * 0.5, 0, "", "",
             &hv_WindowHandle);
  HDevWindowStack::Push(hv_WindowHandle);
}

MainWindow::~MainWindow() {
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::on_LoadFile_clicked(bool check) {
  QString fileName = QFileDialog::getOpenFileName(
      this, ("Open File"), QDir::homePath() + "/tdmms_data/design/",
      ("vdwh design file (*.vdwh)"));
  if (!fileName.isEmpty()) {
    QFile file(fileName);
    QString data;
    QStringList rowOfData;
    QStringList rowData;
    data.clear();
    rowOfData.clear();
    rowData.clear();

    if (file.open(QFile::ReadOnly)) {
      data = file.readAll();
      rowOfData = data.split("\n");
      file.close();
    }

    ui.tdmtableWidget->clear();
    ui.tdmtableWidget->setRowCount(0);
    for (int x = 0; x < rowOfData.size() - 1; x++) {
      ui.tdmtableWidget->insertRow(ui.tdmtableWidget->rowCount());
      rowData = rowOfData.at(x).split(";");
      for (int y = 0; y < rowData.size(); y++) {
        QTableWidgetItem *item;
        item = new QTableWidgetItem(QString(rowData[y]));
        ui.tdmtableWidget->setItem(ui.tdmtableWidget->rowCount() - 1, y, item);
        if (y == 0) ui.tdmtableWidget->item(x, 0)->setCheckState(Qt::Checked);
      }
    }
  }
  ui.tdmtableWidget->resizeColumnsToContents();
  ui.tdmtableWidget->resizeRowsToContents();
}

void MainWindow::selectionChangedSlot_stack() {
  int row = ui.tdmtableWidget->currentRow();

  ui.spinBox_PosinChiptray->setValue(
      ui.tdmtableWidget->item(row, 4)->text().toInt());
  ui.lineEdit_Xofs->setText(ui.tdmtableWidget->item(row, 5)->text());
  ui.lineEdit_Yofs->setText(ui.tdmtableWidget->item(row, 6)->text());
  ui.doubleSpinBox_Rotangle->setValue(
      ui.tdmtableWidget->item(row, 7)->text().toDouble() / 10);
  ui.lineEdit_Filename->setText(ui.tdmtableWidget->item(row, 9)->text());
  ui.spinBox_ChiptrayNo->setValue(
      ui.tdmtableWidget->item(row, 2)->text().toInt());
  try {
    ReadImage(&ho_ImagePreview,
              ui.tdmtableWidget->item(row, 9)->text().toStdString().c_str());
    HDevWindowStack::SetActive(hv_WindowHandle);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_ImagePreview, HDevWindowStack::GetActive());
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    HTuple hv_message;
    GetErrorText((long)(HDevExpDefaultException.ErrorCode()), &hv_message);
    ROS_ERROR("Halcon exception caught");
    std::cout << hv_message << "\n";
  }
}

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

void MainWindow::on_Button_SetPicFolder_clicked(bool check) {
  QFileDialog::Options options =
      QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly;

  QString strDir = QFileDialog::getExistingDirectory(
      0, tr("Folder to Store Image"), QDir::homePath() + "/tdmms_data/log",
      options);

  if (!strDir.isEmpty()) {
    ui.lineEdit_picFolder->setText(strDir);
  }
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() { ui.view_logging->scrollToBottom(); }

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
  QSettings settings("Qt-Ros Package", "tdmms_autostamp_master");
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

  QString foldername = settings.value("folder", QString("")).toString();
  ui.lineEdit_picFolder->setText(foldername);

  int sample_z_stage_pos_w_alignment =
      settings.value("sample_z_stage_pos_w_alignment", 585000).toInt();
  ui.spinBox_Auto_Stamp_ZatStamp->setValue(sample_z_stage_pos_w_alignment);

  int om_at_stamp = settings.value("om_at_stamp", 47380).toInt();
  ui.spinBox_Auto_Stamp_OMatStamp->setValue(om_at_stamp);

  int up_speed = settings.value("up_speed", 10).toInt();
  ui.spinBox_Auto_Stamp_Up_Speed->setValue(up_speed);

  int down_speed = settings.value("down_speed", 1000).toInt();
  ui.spinBox_Auto_Stamp_Down_Speed->setValue(down_speed);

  int stamp_time = settings.value("stamp_time", 10).toInt();
  ui.spinBox_Auto_Stamp_Time->setValue(stamp_time);

  int stamp_load = settings.value("stamp_load", 200).toInt();
  ui.spinBox_Auto_Stamp_Load->setValue(stamp_load);

  int palletNo_1 = settings.value("PalletNo_1", 0).toInt();
  ui.spinBox_PalletNo_1->setValue(palletNo_1);

  int palletNo_2 = settings.value("PalletNo_2", 0).toInt();
  ui.spinBox_PalletNo_2->setValue(palletNo_2);

  int palletNo_3 = settings.value("PalletNo_3", 0).toInt();
  ui.spinBox_PalletNo_3->setValue(palletNo_3);
  
  int chiptrayNo_1 = settings.value("ChiptrayNo_1", 0).toInt();
  ui.spinBox_ChiptrayNo_1->setValue(chiptrayNo_1);

  int chiptrayNo_2 = settings.value("ChiptrayNo_2", 0).toInt();
  ui.spinBox_ChiptrayNo_2->setValue(chiptrayNo_2);

  int chiptrayNo_3 = settings.value("ChiptrayNo_3", 0).toInt();
  ui.spinBox_ChiptrayNo_3->setValue(chiptrayNo_3);
  
  int AlignOffset_theta_x = settings.value("Align_Offset_Theta_X", 85).toInt();
  ui.spinBox_Align_Offset_Theta_X->setValue(AlignOffset_theta_x);

  int AlignOffset_theta_y = settings.value("Align_Offset_Theta_Y", 100).toInt();
  ui.spinBox_Align_Offset_Theta_Y->setValue(AlignOffset_theta_y);

  int AlignOffset_x = settings.value("Align_Offset_X", 85).toInt();
  ui.spinBox_Align_Offset_X->setValue(AlignOffset_x);

  int AlignOffset_y = settings.value("Align_Offset_Y", 100).toInt();
  ui.spinBox_Align_Offset_Y->setValue(AlignOffset_y);

  double AlignScale = settings.value("Align_Scale", 2.4).toDouble();
  ui.doubleSpinBox_Align_Offset_Scale->setValue(AlignScale);

  int Autofocus_Start = settings.value("Autofocus_Start", 35020).toInt();
  ui.spinBox_Autofocus_Start->setValue(Autofocus_Start);

  int Autofocus_Count = settings.value("Autofocus_Count", 80).toInt();
  ui.spinBox_Autofocus_Count->setValue(Autofocus_Count);

  int Autofocus_Step = settings.value("Autofocus_Step", 100).toInt();
  ui.spinBox_Autofocus_Step->setValue(Autofocus_Step);

  int Autofocus_Zpos = settings.value("Autofocus_Zpos", 39020).toInt();
  ui.spinBox_Autofocus_Zpos->setValue(Autofocus_Zpos);

  double Align_NCC_Scale_x =
      settings.value("Align_NCC_Scale_X", 1.78).toDouble();
  ui.doubleSpinBox_Align_NCC_Scale_X->setValue(Align_NCC_Scale_x);

  double Align_NCC_Scale_y =
      settings.value("Align_NCC_Scale_Y", 2.03).toDouble();
  ui.doubleSpinBox_Align_NCC_Scale_Y->setValue(Align_NCC_Scale_y);
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_autostamp_master");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  // settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));

  settings.setValue("folder", QVariant(ui.lineEdit_picFolder->text()));
  settings.setValue("sample_z_stage_pos_w_alignment",
                    QVariant(ui.spinBox_Auto_Stamp_ZatStamp->value()));
  settings.setValue("om_at_stamp",
                    QVariant(ui.spinBox_Auto_Stamp_OMatStamp->value()));
  settings.setValue("up_speed",
                    QVariant(ui.spinBox_Auto_Stamp_Up_Speed->value()));
  settings.setValue("down_speed",
                    QVariant(ui.spinBox_Auto_Stamp_Down_Speed->value()));
  settings.setValue("stamp_time",
                    QVariant(ui.spinBox_Auto_Stamp_Time->value()));
  settings.setValue("stamp_load",
                    QVariant(ui.spinBox_Auto_Stamp_Load->value()));
  settings.setValue("PalletNo_1", QVariant(ui.spinBox_PalletNo_1->value()));
  settings.setValue("PalletNo_2", QVariant(ui.spinBox_PalletNo_2->value()));
  settings.setValue("PalletNo_3", QVariant(ui.spinBox_PalletNo_3->value()));
  settings.setValue("ChiptrayNo_1", QVariant(ui.spinBox_ChiptrayNo_1->value()));
  settings.setValue("ChiptrayNo_2", QVariant(ui.spinBox_ChiptrayNo_2->value()));
  settings.setValue("ChiptrayNo_3", QVariant(ui.spinBox_ChiptrayNo_3->value()));
  settings.setValue("Align_Offset_Theta_X",
                    QVariant(ui.spinBox_Align_Offset_Theta_X->value()));
  settings.setValue("Align_Offset_Theta_Y",
                    QVariant(ui.spinBox_Align_Offset_Theta_Y->value()));
  settings.setValue("Align_Offset_X",
                    QVariant(ui.spinBox_Align_Offset_X->value()));
  settings.setValue("Align_Offset_Y",
                    QVariant(ui.spinBox_Align_Offset_Y->value()));
  settings.setValue("Align_Scale",
                    QVariant(ui.doubleSpinBox_Align_Offset_Scale->value()));
  settings.setValue("Autofocus_Start",
                    QVariant(ui.spinBox_Autofocus_Start->value()));
  settings.setValue("Autofocus_Step",
                    QVariant(ui.spinBox_Autofocus_Step->value()));
  settings.setValue("Autofocus_Count",
                    QVariant(ui.spinBox_Autofocus_Count->value()));
  settings.setValue("Autofocus_Zpos",
                    QVariant(ui.spinBox_Autofocus_Zpos->value()));
  settings.setValue("Align_NCC_Scale_Y",
                    QVariant(ui.doubleSpinBox_Align_NCC_Scale_Y->value()));
  settings.setValue("Align_NCC_Scale_X",
                    QVariant(ui.doubleSpinBox_Align_NCC_Scale_X->value()));
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  HDevWindowStack::SetActive(hv_WindowHandle);
  if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
  QMainWindow::closeEvent(event);
}

}  // namespace tdmms_autostamp_master
