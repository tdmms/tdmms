/**
 * @file /include/tdmms_transfer_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tdmms_transfer_master_QNODE_HPP_
#define tdmms_transfer_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QPushButton>
#include <std_msgs/UInt16.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tdmms_transfer_master {

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

  QPushButton *pushButton_Chuck;
  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

public Q_SLOTS:
  void on_right_open_clicked(bool check);
  void on_right_grip_clicked(bool check);
  void on_right_close_clicked(bool check);
  void on_left_open_clicked(bool check);
  void on_left_close_clicked(bool check);
  void on_left_grip_clicked(bool check);
  void on_DIO_clicked(bool check);
  void on_Chuck_clicked(bool check);
 private:
  int init_argc;
  char **init_argv;
  std_msgs::UInt16 statvalue;
  ros::Publisher chatter_publisher;
  ros::Publisher transfer_valve_open_publisher;
  ros::Publisher transfer_valve_close_publisher;

  ros::Publisher iai_gripper_right_open;
  ros::Publisher iai_gripper_right_close;
  ros::Publisher iai_gripper_left_open;
  ros::Publisher iai_gripper_left_close;
  ros::Publisher sample_valve_open;
  ros::Publisher sample_valve_close;
  ros::Publisher tape_valve_open;
  ros::Publisher tape_valve_close;
  ros::ServiceClient adv_dio_write;
  ros::Subscriber dio_subscriber;
  QStringListModel logging_model;
  void dio_callback(const std_msgs::UInt16::ConstPtr &Status);
};

}  // namespace tdmms_transfer_master

#endif /* tdmms_transfer_master_QNODE_HPP_ */
