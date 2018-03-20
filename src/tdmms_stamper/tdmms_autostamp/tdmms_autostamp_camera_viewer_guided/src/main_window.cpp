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
#include "../include/tdmms_autostamp_camera_viewer_guided/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_autostamp_camera_viewer_guided {

using namespace Qt;

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
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing -
                                       // qt-designer should have this already
                                       // hardwired, but often loses it
                                       // (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  QObject::connect(ui.pushButton_SetFolder, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_SetFolder_clicked(bool)));
  QObject::connect(ui.pushButton_SaveFile, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_SaveFile_clicked(bool)));
  QObject::connect(ui.pushButton_Add, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Add_clicked(bool)));
  QObject::connect(ui.pushButton_Delete, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_Delete_clicked(bool)));
  QObject::connect(ui.pushButton_CheckAll, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_CheckAll_clicked(bool)));
  QObject::connect(ui.pushButton_UncheckAll, SIGNAL(clicked(bool)), &qnode,
                   SLOT(on_UncheckAll_clicked(bool)));
  QObject::connect(ui.horizontalSlider_Xofs, SIGNAL(valueChanged(int)), &qnode,
          SLOT(horizontalSlider_Xofs_valueChanged(int)));
  QObject::connect(ui.horizontalSlider_Yofs, SIGNAL(valueChanged(int)), &qnode,
          SLOT(horizontalSlider_Yofs_valueChanged(int)));
  QObject::connect(ui.tableWidget_Edges, SIGNAL(itemSelectionChanged()), &qnode,
          SLOT(selectionChanged()));
  qnode.spinBox_Xofs = ui.spinBox_Xofs;
  qnode.spinBox_Yofs = ui.spinBox_Yofs;
  qnode.doubleSpinBox_Alpha = ui.doubleSpinBox_Alpha;
  qnode.spinBox_Edges_Low = ui.spinBox_Edges_Low;
  qnode.spinBox_Edges_High = ui.spinBox_Edges_High;
  qnode.lineEdit_DataFolder = ui.lineEdit_DataFolder;
  qnode.tableWidget_Edges = ui.tableWidget_Edges;
  qnode.checkBox_ExtractEdge = ui.checkBox_ExtractEdge;
  qnode.horizontalSlider_Xofs = ui.horizontalSlider_Xofs;
  qnode.horizontalSlider_Yofs = ui.horizontalSlider_Yofs;

  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));

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
  QSettings settings("Qt-Ros Package", "tdmms_autostamp_camera_viewer_guided");
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
  QString foldername= settings.value("foldername", "").toString();
  ui.lineEdit_DataFolder->setText(foldername);
  double alpha = settings.value("alpha", 4.00).toDouble();
  ui.doubleSpinBox_Alpha->setValue(alpha);

  int low_th = settings.value("low", 2).toInt();
  ui.spinBox_Edges_Low ->setValue(low_th);

  int high_th = settings.value("high", 3).toInt();
  ui.spinBox_Edges_High->setValue(high_th);
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_autostamp_camera_viewer_guided");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("foldername", ui.lineEdit_DataFolder->text());
  settings.setValue("low", ui.spinBox_Edges_Low->value());
  settings.setValue("high", ui.spinBox_Edges_High->value());
  settings.setValue("alpha", ui.doubleSpinBox_Alpha->value());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace tdmms_autostamp_camera_viewer_guided
