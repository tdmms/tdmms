#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <iostream>     
#include <cstdlib>      
#include <unistd.h>     
#include <cerrno>       
#include <cstring>  
#include <sys/time.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <csignal>
#include "nic_100a/State.h"

class nic_100a_master
{
 public:
  nic_100a_master()
  {
    ros::NodeHandle node;
    cmd_on_sub_ = node.subscribe("/nic_100a/cmd_on", 1, &nic_100a_master::cmd_on_Callback, this);
    cmd_off_sub_ = node.subscribe("/nic_100a/cmd_off", 1, &nic_100a_master::cmd_off_Callback, this);
    cmd_inten_ = node.subscribe("nic_100a/cmd_inten", 1, &nic_100a_master::cmd_inten_Callback, this);
    stat_pub_ = node.advertise<nic_100a::State>("/nic_100a/status",1);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    server.sin_family = AF_INET;
    server.sin_port = htons(10001);
    server.sin_addr.s_addr = inet_addr("192.168.0.101");
    if(connect(sock, (struct sockaddr *)&server, sizeof(server))<0){
      printf("Connection Error\n");
    }   
  }

  ~nic_100a_master()
  {
    close(sock);
  }
  
  void cmd_on_Callback(const std_msgs::Byte &on_msg)
  {
    int n;
    sendbuf[0] = 0x0;
    sendbuf[1] = 0x2;
    sendbuf[2] = '1';
    sendbuf[3] = '3';
    sendbuf[4] = '0';
    sendbuf[5] = '0';
    sendbuf[6] = 0x0;
    sendbuf[7] = 0x3;
    sendbuf[8] = sendbuf[2]^sendbuf[3]^sendbuf[4]^sendbuf[5]^sendbuf[6]^sendbuf[7];

    write(sock, sendbuf, 9);
    n = read(sock, rcvbuf, sizeof(rcvbuf));
    printf("%d, %s\n", n, rcvbuf);
  }
  
  void cmd_off_Callback(const std_msgs::Byte &off_msg)
  {
    int n;
    sendbuf[0] = 0x0;
    sendbuf[1] = 0x2;
    sendbuf[2] = '2';
    sendbuf[3] = '3';
    sendbuf[4] = '0';
    sendbuf[5] = '0';
    sendbuf[6] = 0x0;
    sendbuf[7] = 0x3;
    sendbuf[8] = sendbuf[2]^sendbuf[3]^sendbuf[4]^sendbuf[5]^sendbuf[6]^sendbuf[7];
    write(sock, sendbuf, 9);
    n = read(sock, rcvbuf, sizeof(rcvbuf));
    printf("%d, %s\n", n, rcvbuf);
  }

  void cmd_inten_Callback(const std_msgs::UInt8 &inten_msg)
  {
    int n;
    int intensity;

    sendbuf[0] = 0x0;
    sendbuf[1] = 0x2;
    sprintf(&sendbuf[2], "11%2X", inten_msg.data);
    sendbuf[6] = 0x0;
    sendbuf[7] = 0x3;
    sendbuf[8] = sendbuf[2]^sendbuf[3]^sendbuf[4]^sendbuf[5]^sendbuf[6]^sendbuf[7];
    printf("%4s\n", &sendbuf[2]);
    
    write(sock, sendbuf, 9);
    n = read(sock, rcvbuf, sizeof(rcvbuf));
    printf("%d, %s\n", n, rcvbuf);
    
    sscanf(&rcvbuf[13], "%2X", &intensity);
    printf("%2X\n", intensity);
    stat_val.intensity = intensity;
    stat_pub_.publish(stat_val);
    
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

int main(int argc, char *argv[]){
  ros::init(argc,argv,"nic_100a_master");
  nic_100a_master nic_100a_;
  ros::spin();
  return 0;
}
