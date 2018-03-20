#include <arpa/inet.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#define SERIAL_DATA_LENGTH 4097

class stamper_nikon_master {
 public:
  stamper_nikon_master() {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    server.sin_family = AF_INET;
    server.sin_port = htons(10000);
    server.sin_addr.s_addr = inet_addr("192.168.0.97");
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
      ROS_ERROR("Connection Error");
      exit(-1);
    }
    ros::NodeHandle node;
    cmd_revolv_next_ =
        node.subscribe("/stamper_nikon_master/cmd_revolv_next", 1,
                       &stamper_nikon_master::cmd_revolv_next_Callback, this);
    cmd_revolv_prev_ =
        node.subscribe("/stamper_nikon_master/cmd_revolv_prev", 1,
                       &stamper_nikon_master::cmd_revolv_prev_Callback, this);
    cmd_revolv_set_ =
        node.subscribe("/stamper_nikon_master/cmd_revolv_set", 1,
                       &stamper_nikon_master::cmd_revolv_set_Callback, this);
    cmd_z_u1_ = node.subscribe("/stamper_nikon_master/cmd_z_u1", 1,
                               &stamper_nikon_master::cmd_z_u1_Callback, this);
    cmd_z_u10_ =
        node.subscribe("/stamper_nikon_master/cmd_z_u10", 1,
                       &stamper_nikon_master::cmd_z_u10_Callback, this);
    cmd_z_u20_ =
        node.subscribe("/stamper_nikon_master/cmd_z_u20", 1,
                       &stamper_nikon_master::cmd_z_u20_Callback, this);
    cmd_z_u200_ =
        node.subscribe("/stamper_nikon_master/cmd_z_u200", 1,
                       &stamper_nikon_master::cmd_z_u200_Callback, this);
    cmd_z_u2000_ =
        node.subscribe("/stamper_nikon_master/cmd_z_u2000", 1,
                       &stamper_nikon_master::cmd_z_u2000_Callback, this);
    cmd_z_d1_ = node.subscribe("/stamper_nikon_master/cmd_z_d1", 1,
                               &stamper_nikon_master::cmd_z_d1_Callback, this);
    cmd_z_d10_ =
        node.subscribe("/stamper_nikon_master/cmd_z_d10", 1,
                       &stamper_nikon_master::cmd_z_d10_Callback, this);
    cmd_z_d20_ =
        node.subscribe("/stamper_nikon_master/cmd_z_d20", 1,
                       &stamper_nikon_master::cmd_z_d20_Callback, this);
    cmd_z_d200_ =
        node.subscribe("/stamper_nikon_master/cmd_z_d200", 1,
                       &stamper_nikon_master::cmd_z_d200_Callback, this);
    cmd_z_d2000_ =
        node.subscribe("/stamper_nikon_master/cmd_z_d2000", 1,
                       &stamper_nikon_master::cmd_z_d2000_Callback, this);
    cmd_z_abs_ =
        node.subscribe("/stamper_nikon_master/cmd_z_abs", 1,
                       &stamper_nikon_master::cmd_z_abs_Callback, this);
  }

  ~stamper_nikon_master() {
    close(sock);
  }

  void cmd_z_abs_Callback(const std_msgs::UInt32 &pos) {
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.ABS.%d", pos.data);
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }

  void cmd_revolv_set_Callback(const std_msgs::UInt8 &pos) {
    snprintf(Send_Buf, sizeof(Send_Buf), "REVOLV.%d", pos.data);
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }

  void cmd_revolv_next_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "REVOLV.NEXT");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_revolv_prev_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "REVOLV.PREV");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_u1_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.U.1");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_u10_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.U.10");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_u20_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.U.20");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_u200_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.U.200");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_u2000_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.U.2000");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_d1_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.D.1");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_d10_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.D.10");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_d20_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.D.20");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_d200_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.D.200");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }
  void cmd_z_d2000_Callback(const std_msgs::Empty &emp) {
    int Send_Res;
    snprintf(Send_Buf, sizeof(Send_Buf), "ZDRIVE.D.2000");
    if (write(sock, Send_Buf, strlen(Send_Buf)) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", Send_Buf);
    }
  }

 private:
  ros::Subscriber cmd_revolv_next_;
  ros::Subscriber cmd_revolv_prev_;
  ros::Subscriber cmd_revolv_set_;
  ros::Subscriber cmd_z_u1_;
  ros::Subscriber cmd_z_u10_;
  ros::Subscriber cmd_z_u20_;
  ros::Subscriber cmd_z_u200_;
  ros::Subscriber cmd_z_u2000_;
  ros::Subscriber cmd_z_d1_;
  ros::Subscriber cmd_z_d10_;
  ros::Subscriber cmd_z_d20_;
  ros::Subscriber cmd_z_d200_;
  ros::Subscriber cmd_z_d2000_;
  ros::Subscriber cmd_z_abs_;
  int sock;
  struct sockaddr_in server;

  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "stamper_nikon_master");
  stamper_nikon_master stamper_nikon_;
  ros::spin();
  return 0;
}
