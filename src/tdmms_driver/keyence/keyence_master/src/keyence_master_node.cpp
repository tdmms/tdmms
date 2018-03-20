#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <std_msgs/Empty.h>
#include "keyence_master/Status.h"
#include "keyence_master/Status_msg.h"

#define SERIAL_DATA_LENGTH 4097
#define BAUDRATE B9600
class keyence {
 public:
  keyence() {
    char Send_Dev[] = "/dev/ttyCom4";
    struct termios Send_Newtio;
    int SendLen;

    Send_Fd = open(Send_Dev, O_RDWR | O_NOCTTY);  // Open device
    if (Send_Fd < 0) {
      perror(Send_Dev);
      exit(-1);
    }
    bzero(&Send_Newtio, sizeof(Send_Newtio));  // Clear new attribute
    Send_Newtio.c_cflag = PARENB | BAUDRATE | CRTSCTS | CS8 | CLOCAL |
                          CREAD;  // New attribute assigned
    Send_Newtio.c_iflag = ICRNL;
    Send_Newtio.c_oflag = OCRNL;
    Send_Newtio.c_lflag = 0;
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    if (tcsetattr(Send_Fd, TCSANOW, &Send_Newtio) != 0) {
      ROS_ERROR("tcsetattr Error!");
      exit(-1);
    }

    snprintf(Send_Buf, sizeof(Send_Buf), "CR\r\n");
    SendLen = strlen(Send_Buf);
    Send_Res = write(Send_Fd, Send_Buf, SendLen);  // Write data to device
    if (Send_Res != -1)
      ROS_INFO("Opening Keyence PLC Controller... ");
    else
      ROS_ERROR("write error!!");
    sleep(0.05);
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));

    ros::NodeHandle node;
    cmd_om_fwd_ = node.subscribe("/keyence_master/cmd_om_fwd", 1,
                                 &keyence::cmd_om_fwd_Callback, this);
    cmd_om_back_ = node.subscribe("/keyence_master/cmd_om_back", 1,
                                  &keyence::cmd_om_back_Callback, this);
    cmd_om_origin_ = node.subscribe("/keyence_master/cmd_om_origin", 1,
                                    &keyence::cmd_om_origin_Callback, this);
    cmd_mask_fwd_ = node.subscribe("/keyence_master/cmd_mask_fwd", 1,
                                   &keyence::cmd_mask_fwd_Callback, this);
    cmd_mask_back_ = node.subscribe("/keyence_master/cmd_mask_back", 1,
                                    &keyence::cmd_mask_back_Callback, this);
    cmd_mask_origin_ = node.subscribe("/keyence_master/cmd_mask_origin", 1,
                                      &keyence::cmd_mask_origin_Callback, this);
    cmd_cool_on_ = node.subscribe("/keyence_master/cmd_sample_cool_on", 1,
                                  &keyence::cmd_cool_on_Callback, this);
    cmd_cool_off_ = node.subscribe("/keyence_master/cmd_sample_cool_off", 1,
                                   &keyence::cmd_cool_off_Callback, this);
    cmd_blow_on_ = node.subscribe("/keyence_master/cmd_sample_blow_on", 1,
                                  &keyence::cmd_blow_on_Callback, this);
    cmd_blow_off_ = node.subscribe("/keyence_master/cmd_sample_blow_off", 1,
                                   &keyence::cmd_blow_off_Callback, this);
    cmd_stage_lock_on_ =
        node.subscribe("/keyence_master/cmd_stage_lock_on", 1,
                       &keyence::cmd_stage_lock_on_Callback, this);
    cmd_stage_lock_off_ =
        node.subscribe("/keyence_master/cmd_stage_lock_off", 1,
                       &keyence::cmd_stage_lock_off_Callback, this);
    cmd_mask_chuck_on_ =
        node.subscribe("/keyence_master/cmd_mask_chuck_on", 1,
                       &keyence::cmd_mask_chuck_on_Callback, this);
    cmd_mask_chuck_off_ =
        node.subscribe("/keyence_master/cmd_mask_chuck_off", 1,
                       &keyence::cmd_mask_chuck_off_Callback, this);
    cmd_sample_chuck_on_ =
        node.subscribe("/keyence_master/cmd_sample_vacuum_on", 1,
                       &keyence::cmd_sample_chuck_on_Callback, this);
    cmd_sample_chuck_off_ =
        node.subscribe("/keyence_master/cmd_sample_vacuum_off", 1,
                       &keyence::cmd_sample_chuck_off_Callback, this);
    stat_pub_ =
        node.advertise<keyence_master::Status_msg>("/keyence_master/status", 1);
    service_ = node.advertiseService("/keyence_master/get_status",
                                     &keyence::get_status, this);
  }

  ~keyence() {
    int SendLen;
    snprintf(Send_Buf, sizeof(Send_Buf), "CQ\r\n");
    SendLen = strlen(Send_Buf);
    Send_Res = write(Send_Fd, Send_Buf, SendLen);  // Write data to device
    if (Send_Res != -1)
      ROS_INFO("Closing Keyence PLC Controller...");
    else
      ROS_ERROR("write error!!");
    close(Send_Fd);
  }
  void status_publisher() {
    int sample_cool;
    int sample_blow;
    int mask_chuck;
    int sample_chuck;
    int kyumen_chuck;
    int om_forward;
    int om_moving = 0;
    int om_back;
    int mask_forward;
    int mask_back;
    int mask_moving = 0;
    int esd_forward;
    int esd_back;
    int esd_moving = 0;
    int dummy1;
    int dummy2;
    snprintf(Send_Buf, sizeof(Send_Buf), "RDS R6000 5\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    sscanf(Read_Buf, "%d %d %d %d %d", &sample_cool, &sample_blow, &mask_chuck,
           &sample_chuck, &kyumen_chuck);
    snprintf(Send_Buf, sizeof(Send_Buf), "RDS R000 8\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    sscanf(Read_Buf, "%d %d %d %d %d %d %d %d", &mask_back, &mask_forward,
           &dummy1, &om_back, &om_forward, &dummy2, &esd_back, &esd_forward);
    keyence_master::Status_msg stat;
    stat.sample_cool = static_cast<bool>(sample_cool);
    stat.sample_blow = static_cast<bool>(sample_blow);
    stat.mask_chuck = static_cast<bool>(mask_chuck);
    stat.sample_chuck = static_cast<bool>(sample_chuck);
    stat.kyumen_chuck = static_cast<bool>(kyumen_chuck);
    stat.om_forward = static_cast<bool>(om_forward);
    stat.om_back = static_cast<bool>(om_back);
    stat.mask_forward = static_cast<bool>(mask_forward);
    stat.mask_back = static_cast<bool>(mask_back);
    stat.esd_forward = static_cast<bool>(esd_forward);
    stat.esd_back = static_cast<bool>(esd_back);
    stat_pub_.publish(stat);
  }

  bool get_status(keyence_master::Status::Request &req,
                  keyence_master::Status::Response &res) {
    int sample_cool;
    int sample_blow;
    int mask_chuck;
    int sample_chuck;
    int kyumen_chuck;
    int om_forward;
    int om_moving = 0;
    int om_back;
    int mask_forward;
    int mask_back;
    int mask_moving = 0;
    int esd_forward;
    int esd_back;
    int esd_moving = 0;
    int dummy1;
    int dummy2;
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RDS R6000 5\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    sscanf(Read_Buf, "%d %d %d %d %d", &sample_cool, &sample_blow, &mask_chuck,
           &sample_chuck, &kyumen_chuck);
    ROS_INFO("ServiceCalled:%s", Read_Buf);
    snprintf(Send_Buf, sizeof(Send_Buf), "RDS R000 8\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    sscanf(Read_Buf, "%d %d %d %d %d %d %d %d", &mask_back, &mask_forward,
           &dummy1, &om_back, &om_forward, &dummy2, &esd_back, &esd_forward);

    res.sample_cool = static_cast<bool>(sample_cool);
    res.sample_blow = static_cast<bool>(sample_blow);
    res.mask_chuck = static_cast<bool>(mask_chuck);
    res.sample_chuck = static_cast<bool>(sample_chuck);
    res.kyumen_chuck = static_cast<bool>(kyumen_chuck);
    res.om_forward = static_cast<bool>(om_forward);
    res.om_back = static_cast<bool>(om_back);
    res.mask_forward = static_cast<bool>(mask_forward);
    res.mask_back = static_cast<bool>(mask_back);
    res.esd_forward = static_cast<bool>(esd_forward);
    res.esd_back = static_cast<bool>(esd_back);
    return true;
  }

  void cmd_om_fwd_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8208 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_om_back_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8207 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_om_origin_Callback(const std_msgs::Empty &emp) {}

  void cmd_mask_fwd_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8206 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_mask_back_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8205 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_mask_origin_Callback(const std_msgs::Empty &emp) {}

  void cmd_cool_on_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6000\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 1) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8200 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8200 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_cool_off_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6000\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 0) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8200 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8200 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_blow_on_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6001\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 1) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8201 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8201 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_blow_off_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6001\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 0) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8201 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8201 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_stage_lock_on_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6004\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 1) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8204 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8204 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_stage_lock_off_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6004\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 0) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8204 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8204 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_mask_chuck_on_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6002\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 1) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8202 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8202 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_mask_chuck_off_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6002\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 0) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8202 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8202 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_sample_chuck_on_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6003\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 1) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8203 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8203 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

  void cmd_sample_chuck_off_Callback(const std_msgs::Empty &emp) {
    tcflush(Send_Fd, TCIOFLUSH);  // Flush data writen
    snprintf(Send_Buf, sizeof(Send_Buf), "RD R6003\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.05).sleep();
    Send_Res = read(Send_Fd, Read_Buf, sizeof(Read_Buf));
    int state;
    sscanf(Read_Buf, "%d", &state);
    if (state == 0) return;
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8203 1\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
    ros::Duration(0.5).sleep();
    snprintf(Send_Buf, sizeof(Send_Buf), "WR R8203 0\r\n");
    Send_Res = write(Send_Fd, Send_Buf, strlen(Send_Buf));
  }

 private:
  ros::Subscriber cmd_om_fwd_;
  ros::Subscriber cmd_om_back_;
  ros::Subscriber cmd_om_origin_;
  ros::Subscriber cmd_mask_fwd_;
  ros::Subscriber cmd_mask_back_;
  ros::Subscriber cmd_mask_origin_;
  ros::Subscriber cmd_cool_on_;
  ros::Subscriber cmd_cool_off_;
  ros::Subscriber cmd_blow_on_;
  ros::Subscriber cmd_blow_off_;
  ros::Subscriber cmd_stage_lock_on_;
  ros::Subscriber cmd_stage_lock_off_;
  ros::Subscriber cmd_mask_chuck_on_;
  ros::Subscriber cmd_mask_chuck_off_;
  ros::Subscriber cmd_sample_chuck_on_;
  ros::Subscriber cmd_sample_chuck_off_;
  ros::Publisher stat_pub_;
  ros::ServiceServer service_;
  int Send_Fd, Send_Res;
  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "keyence_master");
  keyence keyence_master_;
  ros::spin();
  return 0;
}
