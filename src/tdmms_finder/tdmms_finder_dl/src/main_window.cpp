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
#include "../include/tdmms_finder_dl/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_finder_dl {
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  map_itemColIndx["class"] = 0;
  map_itemColIndx["class_label"] = 1;
  map_itemColIndx["exposure"] = 2;
  map_itemColIndx["cent_h"] = 3;
  map_itemColIndx["delta_h"] = 4;
  map_itemColIndx["cent_s"] = 5;
  map_itemColIndx["delta_s"] = 6;
  map_itemColIndx["cent_v"] = 7;
  map_itemColIndx["delta_v"] = 8;
  map_itemColIndx["edge_low"] = 9;
  map_itemColIndx["edge_high"] = 10;
  map_itemColIndx["edge_alpha"] = 11;
  map_itemColIndx["entropy_min"] = 12;
  map_itemColIndx["entropy_max"] = 13;
  map_itemColIndx["area_thresh"] = 14;
  map_itemColIndx["hole_thresh"] = 15;
  map_itemColIndx["close_edge_min_amp"] = 16;
  map_itemColIndx["close_edge_max_gap"] = 17;
  map_itemColIndx["dilation"] = 18;
  map_itemColIndx["erosion"] = 19;
  map_itemColIndx["bg_image"] = 20;
  map_itemColIndx["edge_detect"] = 21;
  map_itemColIndx["dilation2"] = 22;
  map_itemColIndx["erosion2"] = 23;

  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to
                     // on_...() callbacks in this class.
  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt()));  // qApp is a global variable for the application

  QObject::connect(ui.Button_CaptureChiptray_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_CaptureChiptray_DL_clicked()));
  QObject::connect(ui.Button_CaptureChiptray_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_CaptureChiptray_RB_clicked()));

  QObject::connect(ui.Button_DetectSiChips_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_DetectSiChips_DL_clicked()));
  QObject::connect(ui.Button_DetectSiChips_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_DetectSiChips_RB_clicked()));

  QObject::connect(ui.Button_CaptureEdge_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_CaptureEdge_DL_clicked()));
  QObject::connect(ui.Button_CaptureEdge_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_CaptureEdge_RB_clicked()));

  QObject::connect(ui.Button_SaveImage, SIGNAL(clicked()), &qnode,
                   SLOT(on_Button_SaveImage_clicked()));

  QObject::connect(ui.Button_CaptureAlignmentImage_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_CaptureAlignmentImage_DL_clicked()));
  QObject::connect(ui.Button_CaptureAlignmentImage_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_CaptureAlignmentImage_RB_clicked()));

  QObject::connect(ui.Button_StartFind_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_StartFind_DL_clicked()));
  QObject::connect(ui.Button_StartFind_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_StartFind_RB_clicked()));

  QObject::connect(ui.Button_InsertSearchRecord_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_InsertSearchRecord_DL_clicked()));
  QObject::connect(ui.Button_InsertSearchRecord_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_InsertSearchRecord_RB_clicked()));

  QObject::connect(ui.Button_SetKey_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_SetKey_DL_clicked()));
  QObject::connect(ui.Button_SetKey_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_SetKey_RB_clicked()));

  QObject::connect(ui.Button_StartSequence_DL, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_StartSequence_DL_clicked()));
  QObject::connect(ui.Button_StartSequence_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_StartSequence_RB_clicked()));

  QObject::connect(ui.Button_OpenTestWindows_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_OpenTestWindows_RB_clicked()));
  QObject::connect(ui.Button_CloseTestWindows_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_CloseTestWindows_RB_clicked()));
  ////////////////////////////////////////////////////
  // Connect spin box and sliders
  ///////////////////////////////////////////////////
  QObject::connect(ui.spinBox_Exposure_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_Exposure_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_Exposure_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_Exposure_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_CentH_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_CentH_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_CentH_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_CentH_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_DeltaH_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_DeltaH_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_DeltaH_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_DeltaH_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_CentS_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_CentS_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_CentS_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_CentS_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_DeltaS_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_DeltaS_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_DeltaS_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_DeltaS_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_CentV_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_CentV_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_CentV_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_CentV_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_DeltaV_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_DeltaV_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_DeltaV_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_DeltaV_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_EdgeLow_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_EdgeLow_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_EdgeLow_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_EdgeLow_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_EdgeHigh_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_EdgeHigh_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_EdgeHigh_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_EdgeHigh_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_EdgeAlpha_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_EdgeAlpha_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_EdgeAlpha_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_EdgeAlpha_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_EntropyMin_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_EntropyMin_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_EntropyMin_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_EntropyMin_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_EntropyMax_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_EntropyMax_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_EntropyMax_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_EntropyMax_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_AreaThresh_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_AreaThresh_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_AreaThresh_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_AreaThresh_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_HoleThresh_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_HoleThresh_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_HoleThresh_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_HoleThresh_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_CloseEdgeMinAmp_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_CloseEdgeMinAmp_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_CloseEdgeMinAmp_RB,
                   SIGNAL(valueChanged(int)),
                   ui.spinBox_CloseEdgeMinAmp_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_CloseEdgeMaxGap_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_CloseEdgeMaxGap_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_CloseEdgeMaxGap_RB,
                   SIGNAL(valueChanged(int)),
                   ui.spinBox_CloseEdgeMaxGap_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_Dilation_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_Dilation_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_Dilation_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_Dilation_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_Erosion_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_Erosion_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_Erosion_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_Erosion_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_Erosion2_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_Erosion2_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_Erosion2_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_Erosion2_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_Dilation2_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_Dilation2_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_Dilation2_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_Dilation2_RB, SLOT(setValue(int)));

  QObject::connect(ui.spinBox_EdgeDetect_RB, SIGNAL(valueChanged(int)),
                   ui.horizontalSlider_EdgeDetect_RB, SLOT(setValue(int)));
  QObject::connect(ui.horizontalSlider_EdgeDetect_RB, SIGNAL(valueChanged(int)),
                   ui.spinBox_EdgeDetect_RB, SLOT(setValue(int)));

  QObject::connect(ui.horizontalSlider_Exposure_RB, SIGNAL(valueChanged(int)),
                   this,
                   SLOT(on_horizontalSlider_Exposure_RB_valueChanged(int)));

  QObject::connect(ui.Button_DeleteParams_RB, SIGNAL(clicked()),
                   this, SLOT(on_button_DeleteParams_RB_clicked()));
  QObject::connect(ui.Button_AddParams_RB, SIGNAL(clicked()),
                   this, SLOT(on_button_AddParams_RB_clicked()));

  QObject::connect(ui.tableWidget_DetectionParams_RB,
                   SIGNAL(itemSelectionChanged()), this,
                   SLOT(selectionChanged_DetectionParams()));

  QObject::connect(ui.Button_SaveBG_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_SaveBG_clicked()));
  QObject::connect(ui.Button_LoadBG_RB, SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_LoadBG_clicked()));

  QObject::connect(ui.Button_SaveSettings_RB, SIGNAL(clicked()),
                    this, SLOT(on_Button_SaveSettings_clicked()));

  QObject::connect(ui.Button_LoadSettings_RB, SIGNAL(clicked()),
                    this, SLOT(on_Button_LoadSettings_clicked()));

  QObject::connect(ui.Button_PrepareManualRegistration_RB, SIGNAL(clicked()),
                   &qnode,
                   SLOT(on_Button_PrepareManualRegistration_RB_clicked()));
  QObject::connect(ui.Button_PrepareManualRegistration_DL, SIGNAL(clicked()),
                   &qnode,
                   SLOT(on_Button_PrepareManualRegistration_DL_clicked()));
  QObject::connect(ui.Button_ManualRegistration_RB,
                   SIGNAL(clicked()),
                   &qnode,
                   SLOT(on_Button_ManualRegistration_RB_clicked()));
  QObject::connect(ui.Button_ManualRegistration_DL,
                   SIGNAL(clicked()),
                   &qnode, SLOT(on_Button_ManualRegistration_DL_clicked()));

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing -
                                       // qt-designer should have this already
                                       // hardwired, but often loses it
                                       // (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));

  qnode.checkBox_TestMode_DL = ui.checkBox_TestMode_DL;
  qnode.checkBox_TestMode_RB = ui.checkBox_TestMode_RB;
  qnode.checkBox_Enhance_DL = ui.checkBox_Enhance_DL;
  qnode.checkBox_Enhance_RB = ui.checkBox_Enhance_RB;

  qnode.tableWidget_SearchArea_DL = ui.tableWidget_SearchArea_DL;
  qnode.tableWidget_SearchArea_RB = ui.tableWidget_SearchArea_RB;
  qnode.tableWidget_DetectionParams_RB = ui.tableWidget_DetectionParams_RB;
  qnode.spinBox_ChiptrayID_DL = ui.spinBox_ChiptrayID_DL;
  qnode.spinBox_ChiptrayID_RB = ui.spinBox_ChiptrayID_RB;
  qnode.spinBox_pic_count = ui.spinBox_pic_count;
  qnode.spinBox_StepX = ui.spinBox_StepX;
  qnode.spinBox_StepY = ui.spinBox_StepY;
  qnode.spinBox_Delay = ui.spinBox_Delay;
  qnode.spinBox_AreaThresh_DL = ui.spinBox_AreaThresh_DL;

  qnode.lineEdit_picFolder = ui.lineEdit_picFolder;
  qnode.lineEdit_Folder_DL = ui.lineEdit_Folder_DL;
  qnode.lineEdit_Folder_RB = ui.lineEdit_Folder_RB;
  qnode.lineEdit_S3Bucket = ui.lineEdit_S3Bucket;
  qnode.lineEdit_BGImage = ui.lineEdit_BGImage;
  qnode.map_itemColIndx = &map_itemColIndx;

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

void MainWindow::on_Button_SaveSettings_clicked() {
  QFileDialog::Options options;
  QString strSelectedFilter;
  QString strFName = QFileDialog::getSaveFileName(
      this, tr("Save File"),
      QDir::homePath()+"/tdmms_data/detection_params/untitled.prm",
      tr("vdwh design file (*.prm)"));

  if (!strFName.isEmpty()) {
    QFile f(strFName);
    if (f.open(QFile::WriteOnly | QFile::Truncate)) {
      QTextStream data(&f);
      QStringList strList;
      data << ui.lineEdit_BGImage->text() + "\n";
      for (int r = 0; r < ui.tableWidget_DetectionParams_RB->rowCount(); ++r) {
        strList.clear();
        for (int c = 0;
             c < ui.tableWidget_DetectionParams_RB->columnCount();
             ++c) {
          strList << ui.tableWidget_DetectionParams_RB->item(r, c)->text();
        }
        data << strList.join(";") + "\n";
      }
      f.close();
    }
  }
}

void MainWindow::on_Button_LoadSettings_clicked() {
  QString fileName = QFileDialog::getOpenFileName(
      this, ("Open File"), QDir::homePath() + "/tdmms_data/detection_params/",
      ("vdwh design file (*.prm)"));
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

    ui.lineEdit_BGImage->setText(rowOfData.at(0));
    ui.tableWidget_DetectionParams_RB->clear();
    ui.tableWidget_DetectionParams_RB->setRowCount(0);
    for (int x = 1; x < rowOfData.size() - 1; x++) {
      ui.tableWidget_DetectionParams_RB->
          insertRow(ui.tableWidget_DetectionParams_RB->rowCount());
      rowData = rowOfData.at(x).split(";");
      for (int y = 0; y < rowData.size(); y++) {
        QTableWidgetItem *item;
        item = new QTableWidgetItem(QString(rowData[y]));
        ui.tableWidget_DetectionParams_RB->
            setItem(ui.tableWidget_DetectionParams_RB->rowCount() - 1, y, item);
      }
    }
  }
  ui.tableWidget_DetectionParams_RB->resizeColumnsToContents();
  ui.tableWidget_DetectionParams_RB->resizeRowsToContents();
  qnode.prepareBGImage(ui.lineEdit_BGImage->text());
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
}

void MainWindow::selectionChanged_DetectionParams() {
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.spinBox_Exposure_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["exposure"])->text().toInt());
  ui.spinBox_CentH_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["cent_h"])->text().toInt());
  ui.spinBox_DeltaH_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["delta_h"])->text().toInt());
  ui.spinBox_CentS_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["cent_s"])->text().toInt());
  ui.spinBox_DeltaS_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["delta_s"])->text().toInt());
  ui.spinBox_CentV_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["cent_v"])->text().toInt());
  ui.spinBox_DeltaV_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["delta_v"])->text().toInt());
  ui.spinBox_EdgeLow_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["edge_low"])->text().toInt());
  ui.spinBox_EdgeHigh_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["edge_high"])->text().toInt());
  ui.spinBox_EdgeAlpha_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["edge_alpha"])->text().toInt());
  ui.spinBox_EntropyMin_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["entropy_min"])->text().toInt());
  ui.spinBox_EntropyMax_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["entropy_max"])->text().toInt());
  ui.spinBox_AreaThresh_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["area_thresh"])->text().toInt());
  ui.spinBox_HoleThresh_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["hole_thresh"])->text().toInt());
  ui.spinBox_CloseEdgeMinAmp_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["close_edge_min_amp"])->text().toInt());
  ui.spinBox_CloseEdgeMaxGap_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["close_edge_max_gap"])->text().toInt());
  ui.spinBox_Dilation_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["dilation"])->text().toInt());
  ui.spinBox_Erosion_RB->setValue(
      ui.tableWidget_DetectionParams_RB
      ->item(row, map_itemColIndx["erosion"])->text().toInt());
}

void MainWindow::on_button_AddParams_RB_clicked() {
  ui.tableWidget_DetectionParams_RB
      ->insertRow(ui.tableWidget_DetectionParams_RB->rowCount());
  ui.tableWidget_DetectionParams_RB->resizeColumnsToContents();
  ui.tableWidget_DetectionParams_RB->resizeRowsToContents();

  int i;
  QTableWidgetItem *item;
  for (i = 0; i < map_itemColIndx.count(); i ++) {
    item = new QTableWidgetItem(QString("0"));
    ui.tableWidget_DetectionParams_RB
        ->setItem(ui.tableWidget_DetectionParams_RB->rowCount() - 1, i, item);
  }
  int row = ui.tableWidget_DetectionParams_RB->rowCount() - 1;

  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["class"])->setText(QString::number(row+1));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["class_label"])->setText("monolayer");
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["exposure"])->setText(QString::number(5000));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["cent_h"])->setText(QString::number(-200));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["delta_h"])->setText(QString::number(600));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["cent_s"])->setText(QString::number(-100));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["delta_s"])->setText(QString::number(600));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["cent_v"])->setText(QString::number(-200));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["delta_v"])->setText(QString::number(100));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["edge_low"])->setText(QString::number(20));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["edge_high"])->setText(QString::number(50));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["edge_alpha"])->setText(QString::number(3));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["entropy_min"])->setText(QString::number(0));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["entropy_max"])->setText(QString::number(5));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["area_thresh"])->setText(QString::number(1000));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["hole_thresh"])->setText(QString::number(500));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["close_edge_min_amp"])->setText(QString::number(1));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["close_edge_max_gap"])->setText(QString::number(30));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["dilation"])->setText(QString::number(2));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["erosion"])->setText(QString::number(2));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["bg_image"])->setText(QString("a"));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["edge_detect"])->setText(QString("1"));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["dilation2"])->setText(QString("1"));
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["erosion2"])->setText(QString("1"));
}

void MainWindow::on_button_DeleteParams_RB_clicked() {
  ui.tableWidget_DetectionParams_RB->clearSelection();
  ui.tableWidget_DetectionParams_RB
      ->removeRow(ui.tableWidget_DetectionParams_RB->currentRow());
}

void MainWindow::on_horizontalSlider_Erosion2_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["erosion2"])
      ->setText(QString::number(ui.horizontalSlider_Erosion2_RB->value()));
}
void MainWindow::on_horizontalSlider_Dilation2_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["dilation2"])
      ->setText(QString::number(ui.horizontalSlider_Dilation2_RB->value()));
}
void MainWindow::on_horizontalSlider_EdgeDetect_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["edge_detect"])
      ->setText(QString::number(ui.horizontalSlider_EdgeDetect_RB->value()));
}

void MainWindow::on_horizontalSlider_Exposure_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["exposure"])
      ->setText(QString::number(ui.horizontalSlider_Exposure_RB->value()));
}
void MainWindow::on_horizontalSlider_CentH_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["cent_h"])
      ->setText(QString::number(ui.horizontalSlider_CentH_RB->value()));
}
void MainWindow::on_horizontalSlider_DeltaH_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["delta_h"])
      ->setText(QString::number(ui.horizontalSlider_DeltaH_RB->value()));
}
void MainWindow::on_horizontalSlider_CentS_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["cent_s"])
      ->setText(QString::number(ui.horizontalSlider_CentS_RB->value()));
}
void MainWindow::on_horizontalSlider_DeltaS_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["delta_s"])
      ->setText(QString::number(ui.horizontalSlider_DeltaS_RB->value()));
}
void MainWindow::on_horizontalSlider_CentV_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["cent_v"])
      ->setText(QString::number(ui.horizontalSlider_CentV_RB->value()));
}
void MainWindow::on_horizontalSlider_DeltaV_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["delta_v"])
      ->setText(QString::number(ui.horizontalSlider_DeltaV_RB->value()));
}
void MainWindow::on_horizontalSlider_EdgeLow_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["edge_low"])
      ->setText(QString::number(ui.horizontalSlider_EdgeLow_RB->value()));
}
void MainWindow::on_horizontalSlider_EdgeHigh_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["edge_high"])
      ->setText(QString::number(ui.horizontalSlider_EdgeHigh_RB->value()));
}
void MainWindow::on_horizontalSlider_EdgeAlpha_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["edge_alpha"])
      ->setText(QString::number(ui.horizontalSlider_EdgeAlpha_RB->value()));
}
void MainWindow::on_horizontalSlider_EntropyMin_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["entropy_min"])
      ->setText(QString::number(ui.horizontalSlider_EntropyMin_RB->value()));
}
void MainWindow::on_horizontalSlider_EntropyMax_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["entropy_max"])
      ->setText(QString::number(ui.horizontalSlider_EntropyMax_RB->value()));
}
void MainWindow::on_horizontalSlider_AreaThresh_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["area_thresh"])
      ->setText(QString::number(ui.horizontalSlider_AreaThresh_RB->value()));
}
void MainWindow::on_horizontalSlider_HoleThresh_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["hole_thresh"])
      ->setText(QString::number(ui.horizontalSlider_HoleThresh_RB->value()));
}

void MainWindow::
on_horizontalSlider_CloseEdgeMinAmp_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["close_edge_min_amp"])
      ->setText(
          QString::number(ui.horizontalSlider_CloseEdgeMinAmp_RB->value()));
}

void MainWindow::
on_horizontalSlider_CloseEdgeMaxGap_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(
      row, map_itemColIndx["close_edge_max_gap"])
      ->setText(
          QString::number(ui.horizontalSlider_CloseEdgeMaxGap_RB->value()));
}

void MainWindow::on_horizontalSlider_Dilation_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["dilation"])
      ->setText(QString::number(ui.horizontalSlider_Dilation_RB->value()));
}

void MainWindow::on_horizontalSlider_Erosion_RB_valueChanged(int value) {
  if (ui.tableWidget_DetectionParams_RB->rowCount() == 0) return;
  int row = ui.tableWidget_DetectionParams_RB->currentRow();
  ui.tableWidget_DetectionParams_RB->item(row, map_itemColIndx["erosion"])
      ->setText(QString::number(ui.horizontalSlider_Erosion_RB->value()));
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
  QSettings settings("Qt-Ros Package", "tdmms_finder_dl");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url =
      settings.value("master_url", QString("http://192.168.1.2:11311/"))
          .toString();
  QString host_url =
      settings.value("host_url", QString("192.168.1.3")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if (checked) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
  }
  ui.spinBox_ChiptrayID_DL->setValue(settings.value("dl_chiptray_id").toInt());
  ui.spinBox_ChiptrayID_RB->setValue(settings.value("rb_chiptray_id").toInt());
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_finder_dl");
  settings.setValue("master_url",
                    ui.line_edit_master->text());
  settings.setValue("host_url",
                    ui.line_edit_host->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("dl_chiptray_id",
                    ui.spinBox_ChiptrayID_DL->value());
  settings.setValue("rb_chiptray_id",
                    ui.spinBox_ChiptrayID_RB->value());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace tdmms_finder_dl
