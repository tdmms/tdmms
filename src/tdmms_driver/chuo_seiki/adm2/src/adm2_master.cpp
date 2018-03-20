#include <fcntl.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <adm2/velocity.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#define BAUDRATE B9600
#define SERIAL_DATA_LENGTH 4097

class adm2_master {
 public:
  adm2_master() {
    snprintf(Send_Dev, sizeof(Send_Dev), "/dev/ttyCom1");

    ros::NodeHandle node;
    cmd_stp_move_ = node.subscribe("/adm2_master/cmd_stp_move", 1,
                                   &adm2_master::cmd_stp_move_Callback, this);
    cmd_jog_move_ = node.subscribe("/adm2_master/cmd_jog_move", 1,
                                   &adm2_master::cmd_jog_move_Callback, this);
    cmd_abs_move_ = node.subscribe("/adm2_master/cmd_abs_move", 1,
                                   &adm2_master::cmd_abs_move_Callback, this);
    cmd_stop_ = node.subscribe("/adm2_master/cmd_stop", 1,
                               &adm2_master::cmd_stop_Callback, this);
    cmd_init_ = node.subscribe("/adm2_master/cmd_init", 1,
                               &adm2_master::cmd_init_Callback, this);
    cmd_status_ = node.subscribe("/adm2_master/cmd_stat", 1,
                                 &adm2_master::cmd_status_Callback, this);
    cmd_vel_ = node.subscribe("/adm2_master/cmd_vel", 1,
                              &adm2_master::cmd_vel_Callback, this);
    stat_pub_ = node.advertise<geometry_msgs::Point>("/adm2_master/status", 1);
    service = node.advertiseService("/adm2_master/wait_for_stop",
                                    &adm2_master::wait_for_stop, this);

    Send_Fd = open(Send_Dev, O_RDWR | O_NOCTTY);  // Open device
    if (Send_Fd < 0) {
      ROS_ERROR("Cannot Open Serial Port: %s", Send_Dev);
      exit(-1);
    }

    bzero(&Send_Newtio, sizeof(Send_Newtio));  // Clear new attribute
    Send_Newtio.c_cflag =
        BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;  // New attribute assigned
    Send_Newtio.c_iflag = IGNPAR | ICRNL;
    Send_Newtio.c_oflag = 0;
    Send_Newtio.c_lflag = ICANON;
    tcflush(Send_Fd, TCOFLUSH);  // Flush data writen
    if (tcsetattr(Send_Fd, TCSANOW, &Send_Newtio) != 0) {
      ROS_ERROR("Cannot set serial port attributes");
      exit(-1);
    }
  }

  ~adm2_master() { close(Send_Fd); }

  void cmd_stop_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "L:\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    if (Send_Res == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }

    int i;
    for (i = 0; i < 100; i = i + 1) {
      snprintf(Send_Buf, sizeof(Send_Buf), "Q:2\r\n");
      Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
      if (Send_Res == -1) {
        ROS_ERROR("Command Write Error");
        exit(-1);
      } else {
        ROS_DEBUG("Sent: %s", Send_Buf);
      }

      ros::Duration(0.02).sleep();

      Send_Res = read(Send_Fd, Read_Buf, 255);
      if ((Read_Buf[0] == 'K') && (Read_Buf[2] == 'K')) {
        geometry_msgs::Point po;
        cmd_status_Callback(po);
        break;
      }
    }
    tcflush(Send_Fd, TCIFLUSH);
  }

  void cmd_init_Callback(const std_msgs::Empty &emp) {
    int Send_Res;

    snprintf(Send_Buf, sizeof(Send_Buf), "H:AB\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    if (Send_Res == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }
  }

  void cmd_jog_move_Callback(const geometry_msgs::Point &msgpnt) {
    int Send_Res;
    if (msgpnt.x > 0.0 && msgpnt.y == 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:A+\r\n");
    else if (msgpnt.x < 0.0 && msgpnt.y == 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:A-\r\n");
    else if (msgpnt.x == 0.0 && msgpnt.y > 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:B+\r\n");
    else if (msgpnt.x == 0.0 && msgpnt.y < 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:B-\r\n");
    else if (msgpnt.x > 0.0 && msgpnt.y > 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:A+B+\r\n");
    else if (msgpnt.x < 0.0 && msgpnt.y > 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:A+B-\r\n");
    else if (msgpnt.x < 0.0 && msgpnt.y < 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:A-B-\r\n");
    else if (msgpnt.x > 0.0 && msgpnt.y < 0.0)
      snprintf(Send_Buf, sizeof(Send_Buf), "JGO:A+B-\r\n");

    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }
  }

  void cmd_abs_move_Callback(const geometry_msgs::Point &msgpnt) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "AGO:A%08.0fB%08.0f\n", msgpnt.x,
             msgpnt.y);
    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }

    int i;
    for (i = 0; i < 100; i = i + 1) {
      snprintf(Send_Buf, sizeof(Send_Buf), "Q:2\r\n");
      tcflush(Send_Fd, TCIFLUSH);
      Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
      ros::Duration(0.02).sleep();
      Send_Res = read(Send_Fd, Read_Buf, 255);

      if ((Read_Buf[0] == 'K') && (Read_Buf[2] == 'K')) {
        geometry_msgs::Point po;
        cmd_status_Callback(po);
        break;
      }
    }
  }

  void cmd_stp_move_Callback(const geometry_msgs::Point &msgpnt) {
    int Send_Res;
    int i;

    snprintf(Send_Buf, sizeof(Send_Buf), "MGO:A%08.0fB%08.0f\r\n", msgpnt.x,
             msgpnt.y);
    if (write(Send_Fd, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }

    for (i = 0; i < 100; i = i + 1) {
      snprintf(Send_Buf, sizeof(Send_Buf), "Q:2\r\n");

      tcflush(Send_Fd, TCIFLUSH);

      Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
      ros::Duration(0.02).sleep();
      Send_Res = read(Send_Fd, Read_Buf, 255);

      if ((Read_Buf[0] == 'K') && (Read_Buf[2] == 'K')) {
        geometry_msgs::Point po;
        cmd_status_Callback(po);
        break;
      }
    }
  }

  void cmd_status_Callback(const geometry_msgs::Point &msgpnt) {
    int Send_Res;

    snprintf(Send_Buf, sizeof(Send_Buf), "Q:1\r\n");

    tcflush(Send_Fd, TCIFLUSH);
    tcflush(Send_Fd, TCOFLUSH);

    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.02).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));

    geometry_msgs::Point currpoint;
    sscanf(Read_Buf, "%lf,%lf", &currpoint.x, &currpoint.y);
    stat_pub_.publish(currpoint);
  }

  void cmd_vel_Callback(const adm2::velocity vel) {
    int Send_Res;

    snprintf(Send_Buf, sizeof(Send_Buf), "D:A%d,%d,%d\r\n", vel.velocity_low,
             vel.velocity_high, vel.accl);
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    if (Send_Res == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }

    tcflush(Send_Fd, TCIFLUSH);

    snprintf(Send_Buf, sizeof(Send_Buf), "D:B%d,%d,%d\r\n", vel.velocity_low,
             vel.velocity_high, vel.accl);
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    if (Send_Res == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_DEBUG("Sent: %s", Send_Buf);
    }
  }

  bool wait_for_stop(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &resp) {
    int Send_Res;
    ROS_DEBUG("wait for stop");
    do {
      snprintf(Send_Buf, sizeof(Send_Buf), "Q:2\r\n");
      tcflush(Send_Fd, TCIFLUSH);
      ros::Duration(0.1).sleep();
      Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
      ros::Duration(0.1).sleep();
      Send_Res = read(Send_Fd, Read_Buf, 255);
      printf("%s\n", Read_Buf);
    } while ((Read_Buf[0] != 'K') || (Read_Buf[2] != 'K'));
    return true;
  }

 private:
  ros::Subscriber cmd_stp_move_;
  ros::Subscriber cmd_jog_move_;
  ros::Subscriber cmd_abs_move_;
  ros::Subscriber cmd_stop_;
  ros::Subscriber cmd_init_;
  ros::Subscriber cmd_status_;
  ros::Subscriber cmd_vel_;
  ros::Publisher stat_pub_;
  ros::ServiceServer service;
  int Send_Fd;
  char Send_Dev[100];
  char Send_Buf[256];
  char Read_Buf[256];
  struct termios Send_Newtio;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "adm2_master");
  adm2_master adm2_master_;
  ros::spin();
  return 0;
}
