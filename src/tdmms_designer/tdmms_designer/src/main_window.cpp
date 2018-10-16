////////////////////////////////////
/// Copyright 2016.08 by S. Masubuchi
///////////////////////////////////


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
#include <iostream>
#include <QDir>

#include "../include/tdmms_designer/main_window.hpp"

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

namespace tdmms_designer {
using namespace Qt;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
                   SLOT(aboutQt()));
  QObject::connect(ui.submitButton, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonSubmit_clicked(bool)));
  QObject::connect(ui.Button_Add, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonAdd_clicked(bool)));
  QObject::connect(ui.Button_Delete, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonDelete_clicked(bool)));
  QObject::connect(ui.Button_LoadFile, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonLoad_clicked(bool)));
  QObject::connect(ui.Button_SaveFile, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonSave_clicked(bool)));
  QObject::connect(ui.Button_SubmittoStamper, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonSubmittostamper_clicked(bool)));
  QObject::connect(ui.spinBox_EdgeFilter_Low, SIGNAL(valueChanged(int)), this,
                   SLOT(updateStackedImage()));
  QObject::connect(ui.spinBox_EdgeFilter_High, SIGNAL(valueChanged(int)), this,
                   SLOT(updateStackedImage()));
  QObject::connect(ui.doubleSpinBox_EdgeFilter_Alpha,
                   SIGNAL(valueChanged(double)), this,
                   SLOT(updateStackedImage()));
  QObject::connect(ui.Button_EdgeFitting, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonEdgeFitting_clicked(bool)));
  QObject::connect(ui.Button_ClearEdgeFitting, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonClearEdgeFitting_clicked(bool)));
  QObject::connect(ui.comboBox_EdgeFilter, SIGNAL(currentIndexChanged(int)),
                   this, SLOT(onComboboxEdgefilter_currentindex_changed(int)));
  QObject::connect(ui.Button_Checkout, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonCheckout_clicked(bool)));
  QObject::connect(ui.Button_UnCheckout, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonUnCheckout_clicked(bool)));
  QObject::connect(ui.Button_ReloadImage, SIGNAL(clicked(bool)), this,
                   SLOT(onReloadImage_clicked(bool)));
  QObject::connect(ui.Button_Move_Up, SIGNAL(clicked(bool)), this,
                   SLOT(onMoveUp_clicked(bool)));
  QObject::connect(ui.Button_Move_Down, SIGNAL(clicked(bool)), this,
                   SLOT(onMoveDown_clicked(bool)));
  ReadSettings();
  setWindowIcon(QIcon(":/images/1462283158_pen.png"));
  ui.tab_manager->setCurrentIndex(0);
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));

  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }

  // SQLサーバーへ接続
  QSqlError err;
  if (QSqlDatabase::drivers().isEmpty())
    QMessageBox::information(this, tr("No database drivers found"),
                             tr("This demo requires at least"
                                "one Qt database driver. "
                                "Please check the documentation"
                                "how to build the "
                                "Qt SQL plugins."));
  db =
      QSqlDatabase::addDatabase(QString("QMYSQL"), QString("Browser%1").arg(1));
  db.setDatabaseName(QString("mydb"));
  db.setHostName(QString("localhost"));
  db.setPort(3306);
  if (!db.open(QString("root"), QString("mlab2dmms"))) {
    err = db.lastError();
    db = QSqlDatabase();
    QSqlDatabase::removeDatabase(QString("Browser%1").arg(1));
  } else {
    printf("connected to MYSQL Server \n");
  }

  //  Activate GPU
  HTuple hv_Index;

  QueryAvailableComputeDevices(&hv_DeviceIdentifiers);
  {
    HTuple end_val3 = (hv_DeviceIdentifiers.TupleLength()) - 1;
    HTuple step_val3 = 1;
    for (hv_Index = 0; hv_Index.Continue(end_val3, step_val3);
         hv_Index += step_val3) {
      GetComputeDeviceInfo(HTuple(hv_DeviceIdentifiers[hv_Index]), "name",
                           &hv_DeviceName);
      GetComputeDeviceInfo(HTuple(hv_DeviceIdentifiers[hv_Index]), "vendor",
                           &hv_DeviceVendor);
      OpenComputeDevice(HTuple(hv_DeviceIdentifiers[hv_Index]),
                        &hv_DeviceHandle);
    }
  }
  ActivateComputeDevice(hv_DeviceHandle);

  HTuple hv_homedir;
  TupleEnvironment("HOME", &hv_homedir);

  ReadImage(&ho_PreviewImage,
            hv_homedir + "/tdmms_data/images/01/"
            "o_0000000001_-000019992_0000014657_0000000000.jpg");
  GetImageSize(ho_PreviewImage, &hv_Width, &hv_Height);
  hv_syukushaku_preview = 0.7;
  //  Window Open
  SetWindowAttr("window_title", "Preview Image");
  OpenWindow(0, 0, hv_Width * hv_syukushaku_preview,
             hv_Height * hv_syukushaku_preview, 0, "", "",
             &hv_WindowHandlePreview);
  HDevWindowStack::Push(hv_WindowHandlePreview);

  SetWindowAttr("window_title", "Stacked Image");
  OpenWindow(0, 1048, hv_Width * hv_syukushaku_preview,
             hv_Height * hv_syukushaku_preview, 0, "", "",
             &hv_WindowHandleStacked);
  HDevWindowStack::Push((hv_WindowHandleStacked));

  SetWindowAttr("window_title", "Edge Image");
  OpenWindow(600, 0, hv_Width, hv_Height, 0, "", "",
             &hv_WindowHandleDiff);
  HDevWindowStack::Push((hv_WindowHandleDiff));
}

MainWindow::~MainWindow() {
  HDevWindowStack::Pop();
  CloseWindow(hv_WindowHandleDiff);

  HDevWindowStack::Pop();
  CloseWindow(hv_WindowHandleStacked);

  HDevWindowStack::Pop();
  CloseWindow(hv_WindowHandlePreview);
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::cellEnteredSlot(int row, int column){
}

void MainWindow::move(bool up) {
  Q_ASSERT(ui.tdmtableWidget->selectedItems().count() > 0);
  const int sourceRow = ui.tdmtableWidget->row(
      ui.tdmtableWidget->selectedItems().at(0));
  const int destRow = (up ? sourceRow-1 : sourceRow+1);
  Q_ASSERT(destRow >= 0 && destRow < ui.tdmtableWidget->rowCount());

  // take whole rows
  QList<QTableWidgetItem*> sourceItems = this->takeRow(sourceRow);
  QList<QTableWidgetItem*> destItems = this->takeRow(destRow);

  // set back in reverse order
  this->setRow(sourceRow, destItems);
  this->setRow(destRow, sourceItems);
  ui.tdmtableWidget->selectRow(destRow);
}

// takes and returns the whole row
QList<QTableWidgetItem*> MainWindow::takeRow(int row) {
  QList<QTableWidgetItem*> rowItems;
  for (int col = 0; col < ui.tdmtableWidget->columnCount(); ++col) {
    rowItems << ui.tdmtableWidget->takeItem(row, col);
  }
  return rowItems;
}

// sets the whole row
void MainWindow::setRow(int row, const QList<QTableWidgetItem*>& rowItems) {
  for (int col = 0; col < ui.tdmtableWidget->columnCount(); ++col) {
    ui.tdmtableWidget->setItem(row, col, rowItems.at(col));
  }
}

void MainWindow::onMoveUp_clicked(bool check) {
  if(ui.tdmtableWidget->row(ui.tdmtableWidget->selectedItems().at(0)) == 0) return;
  this->move(true);
}

void MainWindow::onMoveDown_clicked(bool check) {
  if(ui.tdmtableWidget->row(ui.tdmtableWidget->selectedItems().at(0)) >= ui.tdmtableWidget->rowCount() -1) return;
  this->move(false);
}

void MainWindow::onReloadImage_clicked(bool check) {
  reloadImage();
}

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::onButtonClearEdgeFitting_clicked(bool check) {
  HTuple empty;

  hv_RowBegin_Tuple = empty;
  hv_ColBegin_Tuple = empty;
  hv_RowEnd_Tuple = empty;
  hv_ColEnd_Tuple = empty;
  updateStackedImage();
}

void MainWindow::onButtonEdgeFitting_clicked(bool check) {
  HTuple hv_Row, hv_Column, hv_Length1, hv_Length2;
  HTuple hv_RowBegin, hv_ColBegin, hv_RowEnd, hv_ColEnd, hv_Nr, hv_Nc, hv_Dist;
  HTuple hv_ColCenter, hv_RowCenter;
  HObject ho_ClippedEdge;
  try {
    DrawRectangle1(hv_WindowHandleDiff, &hv_Row, &hv_Column, &hv_Length1,
                   &hv_Length2);
    ClipContoursXld(ho_Edges_Added, &ho_ClippedEdge, hv_Row, hv_Column,
                    hv_Length1, hv_Length2);

    FitLineContourXld(ho_ClippedEdge, "tukey", -1, 0, 5, 2, &hv_RowBegin,
                      &hv_ColBegin, &hv_RowEnd, &hv_ColEnd, &hv_Nr, &hv_Nc,
                      &hv_Dist);
    hv_ColCenter = (hv_ColBegin + hv_ColEnd) / 2;
    hv_RowCenter = (hv_RowBegin + hv_RowEnd) / 2;
    hv_ColBegin = ((hv_ColBegin - hv_ColCenter) * 100) + hv_ColCenter;
    hv_ColEnd = ((hv_ColEnd - hv_ColCenter) * 100) + hv_ColCenter;
    hv_RowBegin = ((hv_RowBegin - hv_RowCenter) * 100) + hv_RowCenter;
    hv_RowEnd = ((hv_RowEnd - hv_RowCenter) * 100) + hv_RowCenter;
    SetColor(hv_WindowHandleDiff,
             HTuple("red").TupleConcat("green").TupleConcat("blue").TupleConcat(
                 "cyan"));
    DispLine(hv_WindowHandleDiff, hv_RowBegin, hv_ColBegin, hv_RowEnd,
             hv_ColEnd);
    SetColor(hv_WindowHandleDiff, "white");
    hv_RowBegin_Tuple = hv_RowBegin.TupleConcat(hv_RowBegin_Tuple);
    hv_RowEnd_Tuple = hv_RowEnd.TupleConcat(hv_RowBegin_Tuple);
    hv_ColBegin_Tuple = hv_ColBegin.TupleConcat(hv_ColBegin_Tuple);
    hv_ColEnd_Tuple = hv_ColEnd.TupleConcat(hv_ColEnd_Tuple);

    int i;
    for (i = 0; i < hv_RowEnd.TupleLength(); i++) {
      halcon_bridge::disp_message(
          hv_WindowHandleDiff,
          HTuple((hv_Nc[i] / hv_Nr[i]).TupleAtan()).TupleDeg(), "window",
          hv_RowCenter[i], hv_ColCenter[i], "red", "false");
    }
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    HTuple hv_message;
    GetErrorText((long)(HDevExpDefaultException.ErrorCode()), &hv_message);
    ROS_ERROR("Halcon exception caught");
    std::cout << hv_message << "\n";
  }
}

void MainWindow::onButtonSubmit_clicked(bool check) {
  QSqlQueryModel *model = new QSqlQueryModel(ui.table);

  model->setQuery(QSqlQuery(ui.sqlEdit->toPlainText(), db));
  ui.table->setModel(model);

  if (model->lastError().type() != QSqlError::NoError)
    printf("%s\n", model->lastError().text().toStdString().c_str());
  else if (model->query().isSelect())
    printf("%s\n", "Query OK.");
  else
    printf("%s\n", (tr("Query OK, number of affected rows: %1")
                        .arg(model->query().numRowsAffected()))
                       .toStdString()
                       .c_str());

  connect(
      ui.table->selectionModel(),
      SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
      this, SLOT(selectionChangedSlot(const QItemSelection &,
                                      const QItemSelection &)));
  connect(ui.tdmtableWidget, SIGNAL(itemSelectionChanged()), this,
          SLOT(selectionChangedSlot_stack()));
  connect(ui.horizontalSlider_Xofs, SIGNAL(valueChanged(int)), this,
          SLOT(horizontalSlider_Xofs_valueChanged(int)));
  connect(ui.horizontalSlider_Yofs, SIGNAL(valueChanged(int)), this,
          SLOT(horizontalSlider_Yofs_valueChanged(int)));
  connect(ui.horizontalSlider_theta, SIGNAL(valueChanged(int)), this,
          SLOT(horizontalSlider_theta_valueChanged(int)));
  connect(ui.horizontalSlider_alpha, SIGNAL(valueChanged(int)), this,
          SLOT(horizontalSlider_alpha_valueChanged(int)));
  connect(ui.tdmtableWidget, SIGNAL(dropEvent(QDropEvent*)), this,
          SLOT(dropEvent(QDropEvent*)));
  /////// Drag and Drop 対応
  /*  ui.tdmtableWidget->setDragEnabled(true);
  ui.tdmtableWidget->setAcceptDrops(true);
  ui.tdmtableWidget->viewport()->setAcceptDrops(true);
  ui.tdmtableWidget->setDragDropOverwriteMode(false);
  ui.tdmtableWidget->setDropIndicatorShown(true);
   ui.tdmtableWidget->setDragDropMode(QAbstractItemView::InternalMove);*/
  //////////
  
  ui.tdmtableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
  ui.tdmtableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
}

void MainWindow::dropEvent(QDropEvent* event){
  ROS_INFO("aiueo");
  if(event->source() == ui.tdmtableWidget &&
     (event->dropAction() == Qt::MoveAction ||
      ui.tdmtableWidget->dragDropMode() == QAbstractItemView::InternalMove)) {
    bool success;
    int row, col, topIndex;
    success = dropOn(&row, &col, &topIndex);
    if(success){
      int selRow = ui.tdmtableWidget->currentRow();
      int top = selRow;
      int dropRow = row;
      if(dropRow  == -1){
        dropRow = ui.tdmtableWidget->rowCount();
      }
      int offset = dropRow - top;
      int r = row + offset;

      if (r > ui.tdmtableWidget->rowCount() || r < 0) {
        r = 0;
      }
      ui.tdmtableWidget->insertRow(r);

      r = row + offset;
      if(r > ui.tdmtableWidget->rowCount() || r < 0){
        r = 0;
      }

      for(int j = 0; j < ui.tdmtableWidget->columnCount(); j++){
        QTableWidgetItem *source = ui.tdmtableWidget->item(row, j);
        ui.tdmtableWidget->setItem(r, j, source);
      }
      event->accept();
    }
  }
  this->dropEvent(event);
}



bool MainWindow::dropOn(int *row, int *col, int *topIndex){
  
}

void MainWindow::horizontalSlider_Xofs_valueChanged(int value) {
  if (ui.tdmtableWidget->rowCount() == 0) return;
  int row = ui.tdmtableWidget->currentRow();
  ui.tdmtableWidget->item(row, 5)
      ->setText(QString::number(ui.horizontalSlider_Xofs->value()));
  updateStackedImage();
}

void MainWindow::horizontalSlider_Yofs_valueChanged(int value) {
  if (ui.tdmtableWidget->rowCount() == 0) return;
  int row = ui.tdmtableWidget->currentRow();
  ui.tdmtableWidget->item(row, 6)
      ->setText(QString::number(ui.horizontalSlider_Yofs->value()));
  updateStackedImage();
}

void MainWindow::horizontalSlider_theta_valueChanged(int value) {
  if (ui.tdmtableWidget->rowCount() == 0) return;
  int row = ui.tdmtableWidget->currentRow();
  ui.tdmtableWidget->item(row, 7)
      ->setText(QString::number(ui.horizontalSlider_theta->value()));
  updateStackedImage();
}

void MainWindow::horizontalSlider_alpha_valueChanged(int value) {
  if (ui.tdmtableWidget->rowCount() == 0) return;
  int row = ui.tdmtableWidget->currentRow();
  ui.tdmtableWidget->item(row, 8)
      ->setText(QString::number(ui.horizontalSlider_alpha->value()));
  updateStackedImage();
}

void MainWindow::selectionChangedSlot_stack() {
  if (ui.tdmtableWidget->rowCount() == 0) return;
  int row = ui.tdmtableWidget->currentRow();
  ui.horizontalSlider_Xofs->setValue(
      ui.tdmtableWidget->item(row, 5)->text().toInt());
  ui.horizontalSlider_Yofs->setValue(
      ui.tdmtableWidget->item(row, 6)->text().toInt());
  ui.horizontalSlider_theta->setValue(
      ui.tdmtableWidget->item(row, 7)->text().toInt());
  ui.horizontalSlider_alpha->setValue(
      ui.tdmtableWidget->item(row, 8)->text().toInt());
}

void MainWindow::selectionChangedSlot(const QItemSelection &,
                                      const QItemSelection &) {
  const QModelIndex index = ui.table->selectionModel()->currentIndex();
  QString selectedText = index.data(Qt::DisplayRole).toString();
  int row = index.row() == -1 ? 0 : index.row();
  QSqlQueryModel *tm = qobject_cast<QSqlQueryModel *>(ui.table->model());
  filename = tm->data(tm->index(row, 0), Qt::DisplayRole).toString();
  QImage image(filename);
  if (!image.isNull()) {
    ReadImage(&ho_PreviewImage, filename.toStdString().c_str());
    HomMat2dIdentity(&hv_Matrix1);
    HomMat2dScale(hv_Matrix1, 1.0, 1.0, 0, 0, &hv_Matrix2);
    AffineTransImage(ho_PreviewImage, &ho_PreviewImage_trans, hv_Matrix2,
                     "constant", "true");
    HDevWindowStack::SetActive(hv_WindowHandlePreview);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_PreviewImage_trans, HDevWindowStack::GetActive());
  }
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
}

void MainWindow::onButtonDelete_clicked(bool check) {
  ui.tdmtableWidget->clearSelection();
  ui.tdmtableWidget->removeRow(ui.tdmtableWidget->currentRow());
  reloadImage();
  updateStackedImage();
}

void MainWindow::onButtonAdd_clicked(bool check) {
  int id_chip;
  int id_chiptray;
  int position_in_chiptray;

  if (ui.table->model() == NULL) return;

  const QModelIndex index = ui.table->selectionModel()->currentIndex();
  QString selectedText = index.data(Qt::DisplayRole).toString();
  int row = index.row() == -1 ? 0 : index.row();

  ui.tdmtableWidget->insertRow(ui.tdmtableWidget->rowCount());
  ui.tdmtableWidget->resizeColumnsToContents();
  ui.tdmtableWidget->resizeRowsToContents();

  QSqlQueryModel *tm = qobject_cast<QSqlQueryModel *>(ui.table->model());
  filename = tm->data(tm->index(row, 0), Qt::DisplayRole).toString();

  QSqlQuery q("", db);
  q.prepare(
      "select search_id_chip_fk from flake_images where filename_flake_image = "
      "?");
  q.addBindValue(filename);
  if (!q.exec())
    printf("QUERY ERROR, ERROR:%s SENT:%s\n",
           q.lastError().text().toStdString().c_str(),
           q.lastQuery().toStdString().c_str());
  else
    printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());
  q.next();

  id_chip = q.value(0).toInt();
  q.prepare(
      "select id_chiptray_fk, position_in_chiptray from mydb.chip where "
      "id_chip = ?");
  q.addBindValue(id_chip);
  if (!q.exec())
    printf("QUERY ERROR, ERROR:%s SENT:%s\n",
           q.lastError().text().toStdString().c_str(),
           q.lastQuery().toStdString().c_str());
  else
    printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());

  q.next();
  id_chiptray = q.value(0).toInt();
  position_in_chiptray = q.value(1).toInt();

  q.prepare(
      "select name_material_short from "
      "((chip inner join exfoliation_parameter on chip.id_chip=exfoliation_parameter.id_chip)"
      " inner join crystal on id_crystal_fk = id_crystal)"
      " inner join material on id_crystal_material_fk = id_crystal_material"
      " where chip.id_chip = ?");
  q.addBindValue(id_chip);
  if (!q.exec())
    printf("QUERY ERROR, ERROR:%s SENT:%s\n",
           q.lastError().text().toStdString().c_str(),
           q.lastQuery().toStdString().c_str());
  else
    printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());

  QString material_name;
  if(q.next())
    material_name = q.value(0).toString();
  else
    material_name = "NA";

  {
    int i;
    QTableWidgetItem *item;
    for (i = 0; i < 10; i++) {
      item = new QTableWidgetItem(QString("0"));
      item->setFlags(item->flags() & ~(Qt::ItemIsDropEnabled));
      item->setFlags(item->flags() | Qt::ItemIsSelectable);
      ui.tdmtableWidget->setItem(ui.tdmtableWidget->rowCount() - 1, i, item);
    }
    ui.tdmtableWidget->verticalHeader()->setMovable(true);
    ui.tdmtableWidget->setCurrentItem(item);
    int row = ui.tdmtableWidget->rowCount() - 1;
    ui.tdmtableWidget->item(row, 0)->setCheckState(Qt::Checked);
    ui.tdmtableWidget->item(row, 1)->setText(material_name);
    ui.tdmtableWidget->item(row, 2)->setText(QString::number(id_chiptray));
    ui.tdmtableWidget->item(row, 3)->setText(QString::number(id_chip));
    ui.tdmtableWidget->item(row, 4)
        ->setText(QString::number(position_in_chiptray));
    ui.tdmtableWidget->item(row, 5)->setText(tr("0"));
    ui.tdmtableWidget->item(row, 6)->setText(tr("0"));
    ui.tdmtableWidget->item(row, 7)->setText(tr("0"));
    ui.tdmtableWidget->item(row, 8)->setText(tr("50"));
    ui.tdmtableWidget->item(row, 9)->setText(filename);
    ui.tdmtableWidget->setCurrentItem(ui.tdmtableWidget->item(row, 0));
  }
  ui.tdmtableWidget->resizeColumnsToContents();
  ui.tdmtableWidget->resizeRowsToContents();
  reloadImage();
  updateStackedImage();
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

void MainWindow::reloadImage() {
  try{
  GenEmptyObj(&ho_Images);
    for (int i = 0; i < ui.tdmtableWidget->rowCount(); i++) {
      if (ui.tdmtableWidget->item(i, 9)->text() == tr("0")) return;
      ReadImage(
          &ho_Image,
          HTuple(ui.tdmtableWidget->item(i, 9)->text().toStdString().c_str()));
      ConcatObj(ho_Images, ho_Image, &ho_Images);
    }
  } catch (HalconCpp::HException &HDevExpDefaultException) {
    HTuple hv_message;
    GetErrorText((long)(HDevExpDefaultException.ErrorCode()), &hv_message);
    ROS_ERROR("Halcon exception caught");
    std::cout << hv_message << "\n";
  }
}

void MainWindow::updateStackedImage() {
  int i;
  if (ui.tdmtableWidget->rowCount() == 0) {
    return;
  }
  HDevWindowStack::SetActive(hv_WindowHandleDiff);
  HObject ho_Region_Diff;
  HObject ho_Region_Added_diff;
  HTuple hv_Edges_Tuple;
  try {
    GenEmptyObj(&ho_Region_Diff);
    GenEmptyObj(&ho_Region_Added_diff);
    GenEmptyObj(&ho_Edges_Added);
    printf("Update Stacked Image\n");
    GetImageSize(ho_Image, &hv_Width, &hv_Height);
    GenImageConst(&ho_image_bg, "byte", hv_Width * 2, hv_Height * 2);
    Compose3(ho_image_bg, ho_image_bg, ho_image_bg, &ho_image_bg_rgb);
    ho_Image_added = ho_image_bg_rgb;
    ho_Image_added_diff = ho_image_bg_rgb;
    for (i = 0; i < ui.tdmtableWidget->rowCount(); i++) {
      if (ui.tdmtableWidget->item(i, 0)->checkState() == Qt::Checked) {
        hv_xofs = HTuple(ui.tdmtableWidget->item(i, 5)->text().toInt());
        hv_yofs = HTuple(ui.tdmtableWidget->item(i, 6)->text().toInt());
        hv_rot = -HTuple(ui.tdmtableWidget->item(i, 7)->text().toDouble() / 10)
                      .TupleRad();
        hv_alpha =
            HTuple(ui.tdmtableWidget->item(i, 8)->text().toDouble() / 100);

        HomMat2dIdentity(&hv_Matrix1);
        HomMat2dRotate(hv_Matrix1, hv_rot, hv_Height / 2, hv_Width / 2,
                       &hv_Matrix2);
        HomMat2dTranslate(hv_Matrix2, hv_yofs, hv_xofs, &hv_Matrix3);

        SelectObj(ho_Images, &ho_ImageSelected, i + 1);
        AffineTransImageSize(ho_ImageSelected, &ho_ImageSelected_affine,
                             hv_Matrix3, "constant", hv_Width * 2,
                             hv_Height * 2);
        ScaleImage(ho_ImageSelected_affine, &ho_ImageSelected_scaled, hv_alpha,
                   0);
        ScaleImage(ho_Image_added, &ho_Image_Added_scaled, 1 - hv_alpha, 0);
        AddImage(ho_ImageSelected_scaled, ho_Image_Added_scaled,
                 &ho_Image_added, 1, 0);
        if(ui.checkBox_DispEdge->isChecked()){
        switch (ui.comboBox_EdgeFilter->currentIndex()) {
          case 0:
            EdgesColorSubPix(
                ho_ImageSelected_affine, &ho_Edges, "canny",
                ui.doubleSpinBox_EdgeFilter_Alpha->value(),
                static_cast<int>(ui.spinBox_EdgeFilter_Low->value()),
                static_cast<int>(ui.spinBox_EdgeFilter_High->value()));
            break;
          case 1:
            EdgesColorSubPix(
                ho_ImageSelected_affine, &ho_Edges, "deriche1",
                ui.doubleSpinBox_EdgeFilter_Alpha->value(),
                static_cast<int>(ui.spinBox_EdgeFilter_Low->value()),
                static_cast<int>(ui.spinBox_EdgeFilter_High->value()));
            break;
          case 2:
            EdgesColorSubPix(
                ho_ImageSelected_affine, &ho_Edges, "shen",
                ui.doubleSpinBox_EdgeFilter_Alpha->value(),
                static_cast<int>(ui.spinBox_EdgeFilter_Low->value()),
                static_cast<int>(ui.spinBox_EdgeFilter_High->value()));
          default:
            break;
            }
        ho_Edges_Added = ho_Edges.ConcatObj(ho_Edges_Added);
        }
      }
    }

    HDevWindowStack::SetActive(hv_WindowHandleStacked);
    if (HDevWindowStack::IsOpen()){
      DispObj(ho_Image_added, HDevWindowStack::GetActive());
      SetColor(hv_WindowHandleStacked, "red");
      DispLine(HDevWindowStack::GetActive(), 0, hv_Width/2, hv_Height, hv_Width/2);
      DispLine(HDevWindowStack::GetActive(), hv_Height/2, 0, hv_Height/2, hv_Width);
      SetColor(hv_WindowHandleStacked, "green");
      DispLine(HDevWindowStack::GetActive(), 0, hv_Width/2-300, hv_Height, hv_Width/2-300);
      DispLine(HDevWindowStack::GetActive(), hv_Height/2-300, 0, hv_Height/2-300, hv_Width);
      DispLine(HDevWindowStack::GetActive(), 0, hv_Width/2+300, hv_Height, hv_Width/2+300);
      DispLine(HDevWindowStack::GetActive(), hv_Height/2+300, 0, hv_Height/2+300, hv_Width);
    }
    HDevWindowStack::SetActive(hv_WindowHandleDiff);
    ClearWindow(hv_WindowHandleDiff);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_Edges_Added, HDevWindowStack::GetActive());
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    HTuple hv_message;
    GetErrorText((long)(HDevExpDefaultException.ErrorCode()), &hv_message);
    ROS_ERROR("Halcon exception caught");
    std::cout << hv_message << "\n";
  }
}

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
void MainWindow::onButtonUnCheckout_clicked(bool check){
  QString filename;

  for (int r = 0; r < ui.tdmtableWidget->rowCount(); ++r) {
    filename = ui.tdmtableWidget->item(r, 9)->text();

    QSqlQuery q("", db);
    q.prepare(
        "select id_flake_images, objective_lens_id_objective_lens_fk,"
        "table_optical_filter_id_optical_filter_fk, search_id_search,"
        "search_id_chip_fk from flake_images where filename_flake_image = "
        "?");
    q.addBindValue(filename);
    if (!q.exec())
      printf("QUERY ERROR, ERROR:%s SENT:%s\n",
             q.lastError().text().toStdString().c_str(),
             q.lastQuery().toStdString().c_str());
    else
      printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());
    q.next();

    int id_flake_images = q.value(0).toInt();
    int objective_lens_id_objective_lens_fk = q.value(1).toInt();
    int table_optical_filter_id_optical_filter_fk = q.value(2).toInt();
    int search_id_search = q.value(3).toInt();
    int search_id_chip_fk = q.value(4).toInt();

    q.prepare(
        "UPDATE mydb.flake_images SET used='0' WHERE id_flake_images=? and "
        "objective_lens_id_objective_lens_fk=? and "
        "table_optical_filter_id_optical_filter_fk=? and search_id_search=? "
        "and search_id_chip_fk=?");
    q.addBindValue(id_flake_images);
    q.addBindValue(objective_lens_id_objective_lens_fk);
    q.addBindValue(table_optical_filter_id_optical_filter_fk);
    q.addBindValue(search_id_search);
    q.addBindValue(search_id_chip_fk);

    if (!q.exec())
      printf("QUERY ERROR, ERROR:%s SENT:%s\n",
             q.lastError().text().toStdString().c_str(),
             q.lastQuery().toStdString().c_str());
    else
      printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());
  }
}

void MainWindow::onButtonCheckout_clicked(bool check) {
  QString filename;

  for (int r = 0; r < ui.tdmtableWidget->rowCount(); ++r) {
    filename = ui.tdmtableWidget->item(r, 9)->text();

    QSqlQuery q("", db);
    q.prepare(
        "select id_flake_images, objective_lens_id_objective_lens_fk,"
        "table_optical_filter_id_optical_filter_fk, search_id_search,"
        "search_id_chip_fk from flake_images where filename_flake_image = "
        "?");
    q.addBindValue(filename);
    if (!q.exec())
      printf("QUERY ERROR, ERROR:%s SENT:%s\n",
             q.lastError().text().toStdString().c_str(),
             q.lastQuery().toStdString().c_str());
    else
      printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());
    q.next();

    int id_flake_images = q.value(0).toInt();
    int objective_lens_id_objective_lens_fk = q.value(1).toInt();
    int table_optical_filter_id_optical_filter_fk = q.value(2).toInt();
    int search_id_search = q.value(3).toInt();
    int search_id_chip_fk = q.value(4).toInt();

    q.prepare(
        "UPDATE mydb.flake_images SET used='1' WHERE id_flake_images=? and "
        "objective_lens_id_objective_lens_fk=? and "
        "table_optical_filter_id_optical_filter_fk=? and search_id_search=? "
        "and search_id_chip_fk=?");
    q.addBindValue(id_flake_images);
    q.addBindValue(objective_lens_id_objective_lens_fk);
    q.addBindValue(table_optical_filter_id_optical_filter_fk);
    q.addBindValue(search_id_search);
    q.addBindValue(search_id_chip_fk);

    if (!q.exec())
      printf("QUERY ERROR, ERROR:%s SENT:%s\n",
             q.lastError().text().toStdString().c_str(),
             q.lastQuery().toStdString().c_str());
    else
      printf("QUERY SUCCESS, SENT:%s\n", q.lastQuery().toStdString().c_str());
  }
}

void MainWindow::onButtonSave_clicked(bool check) {
  QFileDialog::Options options;
  QString strSelectedFilter;
  QString strFName = QFileDialog::getSaveFileName(
      this, tr("Save File"), QDir::homePath()+"/tdmms_data/design/untitled.vdwh",
      tr("vdwh design file (*.vdwh)"));

  if (!strFName.isEmpty()) {
    QFile f(strFName);
    if (f.open(QFile::WriteOnly | QFile::Truncate)) {
      QTextStream data(&f);
      QStringList strList;
      for (int r = 0; r < ui.tdmtableWidget->rowCount(); ++r) {
        strList.clear();
        for (int c = 0; c < ui.tdmtableWidget->columnCount(); ++c) {
          strList << ui.tdmtableWidget->item(r, c)->text();
        }
        data << strList.join(";") + "\n";
      }
      f.close();
    }
  }
}

void MainWindow::onComboboxEdgefilter_currentindex_changed(int index) {
  updateStackedImage();
}

void MainWindow::onButtonSubmittostamper_clicked(bool check) {
  if (ui.table->model() == NULL) return;

  const QModelIndex index = ui.table->selectionModel()->currentIndex();
  QString selectedText = index.data(Qt::DisplayRole).toString();
  int row = index.row() == -1 ? 0 : index.row();
  QSqlQueryModel *tm = qobject_cast<QSqlQueryModel *>(ui.table->model());
  filename = tm->data(tm->index(row, 0), Qt::DisplayRole).toString();

  qnode.submittostamper(filename);
}

void MainWindow::onButtonLoad_clicked(bool check) {
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
  reloadImage();
  updateStackedImage();
}

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_designer");
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
  QString defquery = settings.value("DefaultQuery", "").toString();
  ui.sqlEdit->setHtml(defquery);
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "tdmms_designer");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  // settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("DefaultQuery",
                    QVariant(ui.sqlEdit->toHtml()));
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace tdmms_designer
