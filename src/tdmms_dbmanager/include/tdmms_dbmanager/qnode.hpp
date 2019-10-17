/**
 * @file /include/tdmms_dbmanager/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tdmms_dbmanager_QNODE_HPP_
#define tdmms_dbmanager_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_dbmanager {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char **argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

  /*********************
  ** Logging
  **********************/
  enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

 private:
  int init_argc;
  char **init_argv;
  ros::Publisher chatter_publisher;
  QStringListModel logging_model;
};

}  // namespace tdmms_dbmanager

#endif /* tdmms_dbmanager_QNODE_HPP_ */
