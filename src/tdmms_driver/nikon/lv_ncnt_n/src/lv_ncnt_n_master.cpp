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
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#define BAUDRATE B9600
#define SERIAL_DATA_LENGTH 4097

class lv_ncnt_n_master {
 public:
  lv_ncnt_n_master() {
    ros::NodeHandle node;
    cmd_set_ = node.subscribe("/lv_ncnt_n_master/cmd_set", 1,
                              &lv_ncnt_n_master::cmd_set_Callback, this);
  }

  ~lv_ncnt_n_master() {}

  void cmd_set_Callback(const std_msgs::UInt8 &revolv) {
    int Send_Fd, Send_Res;
    char Send_Dev[] = "/dev/ttyCom2";

    char Send_Buf[256];
    snprintf(Send_Buf, sizeof(Send_Buf), "cRDC%1d\r\n", revolv.data);
    struct termios Send_Newtio;
    int SendLen;

    Send_Fd = open(Send_Dev, O_RDWR | O_NOCTTY);  // Open device
    if (Send_Fd < 0) {
      perror(Send_Dev);
      exit(-1);
    }
    bzero(&Send_Newtio, sizeof(Send_Newtio));  // Clear new attribute
    Send_Newtio.c_cflag =
        BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;  // New attribute assigned
    Send_Newtio.c_iflag = IGNPAR | ICRNL;
    Send_Newtio.c_oflag = 0;
    Send_Newtio.c_lflag = ICANON;
    tcflush(Send_Fd, TCOFLUSH);                          // Flush data writen
    if (tcsetattr(Send_Fd, TCSANOW, &Send_Newtio) != 0) {
      printf("tcsetattr Error! \n ");
      exit(-1);
    }

    SendLen = strlen(Send_Buf);
    // printf( "Now Sending '%s'+'CR', [%d] bytes ...\n", Send_Buf, SendLen );
    printf("%s\n", Send_Buf);
    Send_Res = write(Send_Fd, Send_Buf, SendLen);  // Write data to device
    if (Send_Res != -1) {
      printf("Sending is over. [%d] bytes.\n", SendLen);
    } else {
      perror("write error!!");
      exit(-1);
    }
    close(Send_Fd);
  }

 private:
  ros::Subscriber cmd_set_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lv_ncnt_n_master");
  lv_ncnt_n_master lv_ncnt_n_master_;
  ros::spin();
  return 0;
}
