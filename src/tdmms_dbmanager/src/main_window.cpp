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
#include "../include/tdmms_dbmanager/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_dbmanager {
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
                   SLOT(aboutQt()));
  QObject::connect(ui.pushButton_AddRecord, SIGNAL(clicked(bool)), this,
                   SLOT(onButtonAddRecord_clicked(bool)));

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0);
  // ensure the first tab is showing - qt-designer should
  // have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

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

  /*********************
   ** Connect to the SQL Server
   **********************/
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
  db.setDatabaseName(QString("2dmms_db"));
  db.setHostName(QProcessEnvironment::systemEnvironment().value("AWS_DB_URL","default"));
  db.setPort(3306);
  if (!db.open(QProcessEnvironment::systemEnvironment().value("AWS_DB_USER","default"),
               QProcessEnvironment::systemEnvironment().value("AWS_DB_PW","default"))) {
    err = db.lastError();
    db = QSqlDatabase();
    QSqlDatabase::removeDatabase(QString("Browser%1").arg(1));
  } else {
    printf("connected to MYSQL Server \n");
  }
  refreshTableWidget();

  /// setCurrentDate
  QDate date = QDate::currentDate();
  ui.dateEdit_Exfoliated->setDate(date);
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
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
void MainWindow::onButtonAddRecord_clicked(bool check) {
  int nofchips = ui.spinBox_NumberofChips->value();
  int id_chiptray = ui.spinBox_TrayID->value();
  QDateTime datetime = ui.dateEdit_Exfoliated->dateTime();
  QString material = ui.lineEdit_MaterialName->text();
  QString exfoliator = ui.lineEdit_ExfoliatorName->text();
  QString substrate = ui.lineEdit_SubstrateName->text();
  QString crystal_name = ui.lineEdit_CrystalName->text();
  QMessageBox msgBox;

  msgBox.setText(tr("Add New Chip Records to RDBMS."));
  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  if (msgBox.exec() == QMessageBox::Cancel) {
    return;
  }

  SQLInsertChip(nofchips, id_chiptray,
                material,
                substrate,
                exfoliator,
                crystal_name,
                datetime);
}

void MainWindow::refreshTableWidget() {
  QSqlQueryModel *model = new QSqlQueryModel(ui.tableView);

  model->setQuery(QSqlQuery(
      "select idchiptray, material, crystal_name, substrate, exfoliator"
      " ,datetime_exfoliated from 2dmms_db.chiptray"
      " inner join 2dmms_db.chip on 2dmms_db.chiptray.idchiptray = 2dmms_db.chip.idchiptray_fk"
      " where position_in_chiptray = 1"
      /*"select "
      " ChipA.id_chiptray_fk, "
      " ChipA.position_in_chiptray, "
      " ChipA.exfoliated_date, "
      " ChipA.exfoliator, "
      " ChipA.material "
      "FROM"
      " mydb.chip AS ChipA "
      " INNER JOIN( "
      " select "
      " id_chiptray_fk, "
      " MAX(position_in_chiptray) AS NumChip "
      " FROM "
      "mydb.chip "
      "GROUP BY "
      " id_chiptray_fk) AS ChipB "
      "ON ChipA.id_chiptray_fk = ChipB.id_chiptray_fk "
      "AND ChipA.position_in_chiptray = ChipB.NumChip;"*/,
      db));
  ui.tableView->setModel(model);

  if (model->lastError().type() != QSqlError::NoError)
    printf("%s\n", model->lastError().text().toStdString().c_str());
  else if (model->query().isSelect())
    printf("%s\n", "Query OK.");
  else
    printf("%s\n", (tr("Query OK, number of affected rows: %1")
                        .arg(model->query().numRowsAffected()))
                       .toStdString()
                       .c_str());
}

void MainWindow::SQLInsertChip(int nofchips, int id_chiptray,
                               QString material,
                               QString substrate,
                               QString exfoliator,
                               QString crystal_name,
                               QDateTime datetime
                               ) {
  QSqlQuery q("", db);
  q.prepare("INSERT INTO 2dmms_db.chiptray (idchiptray) VALUES (?)");
  q.addBindValue(id_chiptray);
  if (!q.exec()) {
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
    ROS_ERROR("The chiptray has already been registered");
    return;
  } else {
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
  }

  int id_chip = SQLGetMaximumChipID();
  int i;
  for (i = 0; i < nofchips; i++) {
    QSqlQuery q("", db);
    id_chip++;
    q.prepare(
        "INSERT INTO 2dmms_db.chip (idchip, idchiptray_fk, material, "
        "substrate, exfoliator, datetime_exfoliated, crystal_name, "
        "position_in_chiptray) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?)");
    q.addBindValue(id_chip);
    q.addBindValue(id_chiptray);
    q.addBindValue(material);
    q.addBindValue(substrate);
    q.addBindValue(exfoliator);
    q.addBindValue(datetime.toString("yyyy-MM-dd"));
    q.addBindValue(crystal_name);
    q.addBindValue(i+1);
    if (!q.exec())
      ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                q.lastError().text().toStdString().c_str(),
                q.executedQuery().toStdString().c_str());
    else
      ROS_DEBUG("QUERY SUCCESS, SENT:%s",
                q.executedQuery().toStdString().c_str());
  }
  refreshTableWidget();
}

int MainWindow::SQLGetMaximumChipID() {
  QSqlQuery q("", db);
  int id_chip;

  q.prepare("SELECT MAX(idchip) FROM 2dmms_db.chip");
  if (!q.exec())
    ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
              q.lastError().text().toStdString().c_str(),
              q.executedQuery().toStdString().c_str());
  else
    ROS_DEBUG("QUERY SUCCESS, SENT:%s",
              q.executedQuery().toStdString().c_str());
  q.next();
  id_chip = q.value(0).toInt();
  return id_chip;
}

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
  QSettings settings("Qt-Ros Package", "tdmms_dbmanager");
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
  QSettings settings("Qt-Ros Package", "tdmms_dbmanager");
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

}  // namespace tdmms_dbmanager
