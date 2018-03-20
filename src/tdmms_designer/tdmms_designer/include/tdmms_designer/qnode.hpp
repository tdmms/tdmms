/**
 * @file /include/tdmms_designer/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tdmms_designer_QNODE_HPP_
#define tdmms_designer_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QString>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_designer {

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
  void submittostamper(QString filename);

 Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();


 private:
  int init_argc;
  char **init_argv;
  ros::Publisher chatter_publisher;
  ros::Publisher filename_publisher;
  QStringListModel logging_model;
};

}  // namespace tdmms_designer

#endif /* tdmms_designer_QNODE_HPP_ */
