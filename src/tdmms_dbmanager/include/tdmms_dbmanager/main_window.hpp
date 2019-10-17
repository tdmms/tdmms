#ifndef tdmms_dbmanager_MAIN_WINDOW_H
#define tdmms_dbmanager_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtSql>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace tdmms_dbmanager {

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

  void ReadSettings();
  void WriteSettings();
  void closeEvent(QCloseEvent *event);
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
  void onButtonAddRecord_clicked(bool check);
  void updateLoggingView();

 private:
  void refreshTableWidget();
  void SQLInsertChip(int nofchips, int id_chiptray,
                     QString material,
                     QString substrate,
                     QString exfoliator,
                     QString crystal_name,
                     QDateTime datetime);
  int SQLGetMaximumChipID();
  Ui::MainWindowDesign ui;
  QString currentDb;
  QSqlDatabase db;
  QNode qnode;
};

}  // namespace tdmms_dbmanager

#endif  // tdmms_dbmanager_MAIN_WINDOW_H
