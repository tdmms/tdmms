#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <arpa/inet.h>
#include <nic_100a/State.h>

class stamper_nic_100a_master {
 public:
  stamper_nic_100a_master() {
    ros::NodeHandle node;
    cmd_on_sub_ =
        node.subscribe("/stamper_nic_100a/cmd_on", 1,
                       &stamper_nic_100a_master::cmd_on_Callback, this);
    cmd_off_sub_ =
        node.subscribe("/stamper_nic_100a/cmd_off", 1,
                       &stamper_nic_100a_master::cmd_off_Callback, this);
    cmd_inten_ =
        node.subscribe("/stamper_nic_100a/cmd_inten", 1,
                       &stamper_nic_100a_master::cmd_inten_Callback, this);
    stat_pub_ = node.advertise<nic_100a::State>("/stamper_nic_100a/status", 1);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    server.sin_family = AF_INET;
    server.sin_port = htons(10001);
    server.sin_addr.s_addr = inet_addr("192.168.0.102");
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
      ROS_ERROR("Connection Error");
      exit(-1);
    } else {
      ROS_INFO("Connected to NIC_100a");
    }
  }

  ~stamper_nic_100a_master() { close(sock); }

  void cmd_on_Callback(const std_msgs::Byte &on_msg) {
    int n;
    sendbuf[0] = 0x0;
    sendbuf[1] = 0x2;
    sendbuf[2] = '1';
    sendbuf[3] = '3';
    sendbuf[4] = '0';
    sendbuf[5] = '0';
    sendbuf[6] = 0x0;
    sendbuf[7] = 0x3;
    sendbuf[8] = sendbuf[2] ^ sendbuf[3] ^ sendbuf[4] ^ sendbuf[5] ^
                 sendbuf[6] ^ sendbuf[7];

    if (write(sock, sendbuf, 9) < 0) {
      ROS_ERROR("Command Write Error: %s", sendbuf);
      exit(-1);
    } else {
      ros::Duration(0.05).sleep();
      n = read(sock, rcvbuf, sizeof(rcvbuf));
      ROS_INFO("Message Recieved:%d, %s", n, rcvbuf);
    }
  }

  void cmd_off_Callback(const std_msgs::Byte &off_msg) {
    int n;
    sendbuf[0] = 0x0;
    sendbuf[1] = 0x2;
    sendbuf[2] = '2';
    sendbuf[3] = '3';
    sendbuf[4] = '0';
    sendbuf[5] = '0';
    sendbuf[6] = 0x0;
    sendbuf[7] = 0x3;
    sendbuf[8] = sendbuf[2] ^ sendbuf[3] ^ sendbuf[4] ^ sendbuf[5] ^
                 sendbuf[6] ^ sendbuf[7];
    if (write(sock, sendbuf, 9) < 0) {
      ROS_ERROR("Command Write Error: %s", sendbuf);
      exit(-1);
    } else {
      ros::Duration(0.05).sleep();
      n = read(sock, rcvbuf, sizeof(rcvbuf));
      ROS_INFO("Message Recieved:%d, %s", n, rcvbuf);
    }
  }

  void cmd_inten_Callback(const std_msgs::UInt8 &inten_msg) {
    int n;
    int intensity;

    sendbuf[0] = 0x0;
    sendbuf[1] = 0x2;
    sprintf(&sendbuf[2], "11%2X", inten_msg.data);
    sendbuf[6] = 0x0;
    sendbuf[7] = 0x3;
    sendbuf[8] = sendbuf[2] ^ sendbuf[3] ^ sendbuf[4] ^ sendbuf[5] ^
                 sendbuf[6] ^ sendbuf[7];

    if (write(sock, sendbuf, 9) < 0) {
      ROS_ERROR("Command Write Error: %s", sendbuf);
      exit(-1);
    } else {
      ros::Duration(0.05).sleep();
      n = read(sock, rcvbuf, sizeof(rcvbuf));
      ROS_INFO("Message Recieved:%d, %s", n, rcvbuf);
      sscanf(&rcvbuf[13], "%2X", &intensity);
      ROS_INFO("Intensity Set to:%d", intensity);
      stat_val.intensity = intensity;
      stat_pub_.publish(stat_val);
    }
  }

 private:
  ros::Subscriber cmd_on_sub_;
  ros::Subscriber cmd_off_sub_;
  ros::Subscriber cmd_inten_;
  ros::Publisher stat_pub_;
  int sock;
  char rcvbuf[23];
  char sendbuf[10];
  struct sockaddr_in server;
  nic_100a::State stat_val;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "stamper_nic_100a_master");
  stamper_nic_100a_master stamper_nic_100a_;
  ros::spin();
  return 0;
}
