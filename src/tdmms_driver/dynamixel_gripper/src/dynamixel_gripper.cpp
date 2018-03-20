// Copyright 2016 S. Masubuchi

#include <arpa/inet.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <string>
#include <iomanip>
#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include "dynamixel.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#define P_GOAL_POSITION_L 30
#define P_GOAL_POSITION_H 31
#define P_PRESENT_POSITION_L 36
#define P_PRESENT_POSITION_H 37
#define P_MOVING 46
#define P_GOAL_TORQUE_L 71
#define P_GOAL_TORQUE_H 72
#define DEFAULT_BAUDNUM 1  // 1Mbps
#define DEFAULT_ID 1

#define ID (2)
#define LENGTH (3)
#define INSTRUCTION (4)
#define ERRBIT (4)
#define PARAMETER (5)
#define DEFAULT_BAUDNUMBER (1)

#define SERIAL_DATA_LENGTH 4097

class dynamixel_gripper_master {
 public:
  void close_port() {
    if (gSocket_fd != -1) close(gSocket_fd);
    gSocket_fd = -1;
  }

  dynamixel_gripper_master() {
    ros::NodeHandle node;
    float baudrate=57143;

    struct termios newtio;
    struct serial_struct serinfo;
    char dev_name[100] = {0, };

    sprintf(dev_name, "/dev/ttyUSBDynamixel");
    strcpy(gDeviceName, dev_name);
    memset(&newtio, 0, sizeof(newtio));
    close_port();

    if ((gSocket_fd = open(gDeviceName, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
      ROS_ERROR("device open error: %s", dev_name);
      exit(-1);
    }

    newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(gSocket_fd, TCIFLUSH);
    tcsetattr(gSocket_fd, TCSANOW, &newtio);

    if (gSocket_fd == -1) exit(-1);

    if (ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
      ROS_ERROR("Cannot get serial info\n");
      exit(-1);
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if (ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
      ROS_ERROR("Cannot set serial info\n");
      exit(-1);
    }

    close_port();

    gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);

    strcpy(gDeviceName, dev_name);
    memset(&newtio, 0, sizeof(newtio));
    close_port();

    if ((gSocket_fd = open(gDeviceName, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
      ROS_ERROR("device open error: %s\n", dev_name);
      exit(-1);
    }

    newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(gSocket_fd, TCIFLUSH);
    tcsetattr(gSocket_fd, TCSANOW, &newtio);

    dxl_hal_set_baud(baudrate);
    gbCommStatus = COMM_RXSUCCESS;
    giBusUsing = 0;

    cmd_left_close_ =
        node.subscribe("/dynamixel_gripper/cmd_left_close", 1,
                       &dynamixel_gripper_master::cmd_left_close_Callback, this);

    cmd_left_open_ =
        node.subscribe("/dynamixel_gripper/cmd_left_open", 1,
                       &dynamixel_gripper_master::cmd_left_open_Callback, this);

    cmd_right_close_ =
        node.subscribe("/dynamixel_gripper/cmd_right_close", 1,
                       &dynamixel_gripper_master::cmd_right_close_Callback, this);

    cmd_right_open_ =
        node.subscribe("/dynamixel_gripper/cmd_right_open", 1,
                       &dynamixel_gripper_master::cmd_right_open_Callback, this);
    dxl_write_byte(1, 24, 1);
    dxl_write_word(1,71, 10);
    dxl_write_word(2,71, 1024+10);
    dxl_write_word(3,71, 10);
    dxl_write_word(4,71, 1024+10);
    
    unsigned int moving_speed;

    int PresentPos = dxl_read_word( 1, P_PRESENT_POSITION_L );
    ROS_DEBUG("%d", PresentPos);
  }

  ~dynamixel_gripper_master() {
    close_port();
  }

  int dxl_hal_set_baud(float baudrate) {
    struct serial_struct serinfo;

    if (gSocket_fd == -1) return 0;

    if (ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
      ROS_ERROR("Cannot get serial info\n");
      return 0;
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if (ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
      ROS_ERROR("Cannot set serial info\n");
      return 0;
    }

    // dxl_hal_close();
    // dxl_hal_open(gDeviceName, baudrate);

    gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);
    return 1;
  }

  void dxl_hal_clear(void) {
    tcflush(gSocket_fd, TCIFLUSH);
  }

  int dxl_hal_tx(unsigned char *pPacket, int numPacket) {
    return write(gSocket_fd, pPacket, numPacket);
  }

  int dxl_hal_rx(unsigned char *pPacket, int numPacket) {
    memset(pPacket, 0, numPacket);
    return read(gSocket_fd, pPacket, numPacket);
  }

  static inline long myclock() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
  }

  void dxl_hal_set_timeout(int NumRcvByte) {
    glStartTime = myclock();
    gfRcvWaitTime = (float)(gfByteTransTime * (float)NumRcvByte + 5.0f);
  }

  int dxl_hal_timeout(void) {
    long time;

    time = myclock() - glStartTime;

    if (time > gfRcvWaitTime)
      return 1;
    else if (time < 0)
      glStartTime = myclock();

    return 0;
  }

  /*int dxl_initialize(int deviceIndex, int baudnum) {
    float baudrate;
    baudrate = 2000000.0f / (float)(baudnum + 1);

    //    if (dxl_hal_open(deviceIndex, baudrate) == 0) return 0;

    return 1;
    }*/

  void dxl_terminate(void) { close_port(); }

  void dxl_tx_packet(void) {
    unsigned char i;
    unsigned char TxNumByte, RealTxNumByte;
    unsigned char checksum = 0;

    if (giBusUsing == 1) return;

    giBusUsing = 1;

    if (gbInstructionPacket[LENGTH] > (MAXNUM_TXPARAM + 2)) {
      gbCommStatus = COMM_TXERROR;
      giBusUsing = 0;
      return;
    }

    if (gbInstructionPacket[INSTRUCTION] != INST_PING &&
        gbInstructionPacket[INSTRUCTION] != INST_READ &&
        gbInstructionPacket[INSTRUCTION] != INST_WRITE &&
        gbInstructionPacket[INSTRUCTION] != INST_REG_WRITE &&
        gbInstructionPacket[INSTRUCTION] != INST_ACTION &&
        gbInstructionPacket[INSTRUCTION] != INST_RESET &&
        gbInstructionPacket[INSTRUCTION] != INST_SYNC_WRITE) {
      gbCommStatus = COMM_TXERROR;
      giBusUsing = 0;
      return;
    }

    gbInstructionPacket[0] = 0xff;
    gbInstructionPacket[1] = 0xff;
    for (i = 0; i < (gbInstructionPacket[LENGTH] + 1); i++)
      checksum += gbInstructionPacket[i + 2];
    gbInstructionPacket[gbInstructionPacket[LENGTH] + 3] = ~checksum;

    if (gbCommStatus == COMM_RXTIMEOUT || gbCommStatus == COMM_RXCORRUPT)
      dxl_hal_clear();

    TxNumByte = gbInstructionPacket[LENGTH] + 4;
    RealTxNumByte = dxl_hal_tx((unsigned char *)gbInstructionPacket, TxNumByte);

    if (TxNumByte != RealTxNumByte) {
      gbCommStatus = COMM_TXFAIL;
      giBusUsing = 0;
      return;
    }

    if (gbInstructionPacket[INSTRUCTION] == INST_READ)
      dxl_hal_set_timeout(gbInstructionPacket[PARAMETER + 1] + 6);
    else
      dxl_hal_set_timeout(6);

    gbCommStatus = COMM_TXSUCCESS;
  }

  void dxl_rx_packet(void) {
    unsigned char i, j, nRead;
    unsigned char checksum = 0;

    if (giBusUsing == 0) return;

    if (gbInstructionPacket[ID] == BROADCAST_ID) {
      gbCommStatus = COMM_RXSUCCESS;
      giBusUsing = 0;
      return;
    }

    if (gbCommStatus == COMM_TXSUCCESS) {
      gbRxGetLength = 0;
      gbRxPacketLength = 6;
    }

    nRead = dxl_hal_rx((unsigned char *)&gbStatusPacket[gbRxGetLength],
                       gbRxPacketLength - gbRxGetLength);
    gbRxGetLength += nRead;
    if (gbRxGetLength < gbRxPacketLength) {
      if (dxl_hal_timeout() == 1) {
        if (gbRxGetLength == 0)
          gbCommStatus = COMM_RXTIMEOUT;
        else
          gbCommStatus = COMM_RXCORRUPT;
        giBusUsing = 0;
        return;
      }
    }

    // Find packet header
    for (i = 0; i < (gbRxGetLength - 1); i++) {
      if (gbStatusPacket[i] == 0xff && gbStatusPacket[i + 1] == 0xff) {
        break;
      } else if (i == gbRxGetLength - 2 &&
                 gbStatusPacket[gbRxGetLength - 1] == 0xff) {
        break;
      }
    }
    if (i > 0) {
      for (j = 0; j < (gbRxGetLength - i); j++)
        gbStatusPacket[j] = gbStatusPacket[j + i];

      gbRxGetLength -= i;
    }

    if (gbRxGetLength < gbRxPacketLength) {
      gbCommStatus = COMM_RXWAITING;
      return;
    }

    // Check id pairing
    if (gbInstructionPacket[ID] != gbStatusPacket[ID]) {
      gbCommStatus = COMM_RXCORRUPT;
      giBusUsing = 0;
      return;
    }

    gbRxPacketLength = gbStatusPacket[LENGTH] + 4;
    if (gbRxGetLength < gbRxPacketLength) {
      nRead = dxl_hal_rx((unsigned char *)&gbStatusPacket[gbRxGetLength],
                         gbRxPacketLength - gbRxGetLength);
      gbRxGetLength += nRead;
      if (gbRxGetLength < gbRxPacketLength) {
        gbCommStatus = COMM_RXWAITING;
        return;
      }
    }

    // Check checksum
    for (i = 0; i < (gbStatusPacket[LENGTH] + 1); i++)
      checksum += gbStatusPacket[i + 2];
    checksum = ~checksum;

    if (gbStatusPacket[gbStatusPacket[LENGTH] + 3] != checksum) {
      gbCommStatus = COMM_RXCORRUPT;
      giBusUsing = 0;
      return;
    }

    gbCommStatus = COMM_RXSUCCESS;
    giBusUsing = 0;
  }

  void dxl_txrx_packet(void) {
    dxl_tx_packet();

    if (gbCommStatus != COMM_TXSUCCESS) return;

    do {
      dxl_rx_packet();
    } while (gbCommStatus == COMM_RXWAITING);
  }

  int dxl_get_result(void) { return gbCommStatus; }

  void dxl_set_txpacket_id(int id) {
    gbInstructionPacket[ID] = (unsigned char)id;
  }

  void dxl_set_txpacket_instruction(int instruction) {
    gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
  }

  void dxl_set_txpacket_parameter(int index, int value) {
    gbInstructionPacket[PARAMETER + index] = (unsigned char)value;
  }

  void dxl_set_txpacket_length(int length) {
    gbInstructionPacket[LENGTH] = (unsigned char)length;
  }

  int dxl_get_rxpacket_error(int errbit) {
    if (gbStatusPacket[ERRBIT] & (unsigned char)errbit) return 1;

    return 0;
  }

  int dxl_get_rxpacket_length(void) { return static_cast<int>(gbStatusPacket[LENGTH]); }

  int dxl_get_rxpacket_parameter(int index) {
    return (int)gbStatusPacket[PARAMETER + index];
  }

  int dxl_makeword(int lowbyte, int highbyte) {
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;
    return (int)word;
  }

  int dxl_get_lowbyte(int word) {
    unsigned short temp;

    temp = word & 0xff;
    return (int)temp;
  }

  int dxl_get_highbyte(int word) {
    unsigned short temp;

    temp = word & 0xff00;
    temp = temp >> 8;
    return (int)temp;
  }

  void dxl_ping(int id) {
    while (giBusUsing)
      ;

    gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = INST_PING;
    gbInstructionPacket[LENGTH] = 2;

    dxl_txrx_packet();
  }

  int dxl_read_byte(int id, int address) {
    while (giBusUsing)
      ;

    gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = INST_READ;
    gbInstructionPacket[PARAMETER] = (unsigned char)address;
    gbInstructionPacket[PARAMETER + 1] = 1;
    gbInstructionPacket[LENGTH] = 4;

    dxl_txrx_packet();

    return (int)gbStatusPacket[PARAMETER];
  }

  void dxl_write_byte(int id, int address, int value) {
    while (giBusUsing)
      ;

    gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = INST_WRITE;
    gbInstructionPacket[PARAMETER] = (unsigned char)address;
    gbInstructionPacket[PARAMETER + 1] = (unsigned char)value;
    gbInstructionPacket[LENGTH] = 4;

    dxl_txrx_packet();
  }

  int dxl_read_word(int id, int address) {
    while (giBusUsing)
      ;

    gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = INST_READ;
    gbInstructionPacket[PARAMETER] = (unsigned char)address;
    gbInstructionPacket[PARAMETER + 1] = 2;
    gbInstructionPacket[LENGTH] = 4;

    dxl_txrx_packet();

    return dxl_makeword((int)gbStatusPacket[PARAMETER],
                        (int)gbStatusPacket[PARAMETER + 1]);
  }

  void dxl_write_word(int id, int address, int value) {
    while (giBusUsing)
      ;

    gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = INST_WRITE;
    gbInstructionPacket[PARAMETER] = (unsigned char)address;
    gbInstructionPacket[PARAMETER + 1] = (unsigned char)dxl_get_lowbyte(value);
    gbInstructionPacket[PARAMETER + 2] = (unsigned char)dxl_get_highbyte(value);
    gbInstructionPacket[LENGTH] = 5;

    dxl_txrx_packet();
  }

  bool sendCommand(const std::string strCommand) {
    strSendBuf = strCommand + strDelimiter;
    if (write(sock, strSendBuf.c_str(), strSendBuf.length()) == -1) {
      ROS_ERROR("Command Write Error");
      exit(-1);
    } else {
      ROS_INFO("Sent: %s", strCommand.c_str());
    }
  }

  std::string readResponse() {
    read(sock, Read_Buf, 255);
    return Read_Buf;
  }

  void cmd_left_open_Callback(const std_msgs::Empty &emp) {
    dxl_write_word(3,71, 10);
    dxl_write_word(4,71, 1024+10);
  }
  void cmd_left_close_Callback(const std_msgs::Empty &emp) {
    dxl_write_word(3,71, 1024 + 10);
    dxl_write_word(4,71, 10);
  }
  void cmd_right_close_Callback(const std_msgs::Empty &emp) {
    dxl_write_word(1,71, 1024 + 10);
    dxl_write_word(2,71, 10);
  }
  void cmd_right_open_Callback(const std_msgs::Empty &emp) {
    dxl_write_word(1,71, 10);
    dxl_write_word(2,71, 1024+10);
  }

  void cmd_open_Callback(const std_msgs::Empty &emp) { sendCommand("OUTP 1"); }

  void cmd_close_Callback(const std_msgs::Empty &emp) { sendCommand("OUTP 0"); }

 private:
  ros::Subscriber cmd_left_open_;
  ros::Subscriber cmd_left_close_;
  ros::Subscriber cmd_right_open_;
  ros::Subscriber cmd_right_close_;
  int sock;
  struct sockaddr_in server;
  char Send_Buf[SERIAL_DATA_LENGTH];
  char Read_Buf[SERIAL_DATA_LENGTH];
  std::string strCommand;
  std::string strSendBuf;
  std::string strDelimiter;

  int gSocket_fd;
  long glStartTime;
  float gfRcvWaitTime;
  float gfByteTransTime;
  char gDeviceName[20];

  unsigned char gbInstructionPacket[MAXNUM_TXPARAM + 10];
  unsigned char gbStatusPacket[MAXNUM_RXPARAM + 10];
  unsigned char gbRxPacketLength;
  unsigned char gbRxGetLength;

  int gbCommStatus;
  int giBusUsing;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dynamixel_gripper_master");
  dynamixel_gripper_master dynamixel_gripper_master_;
  ros::spin();
  return 0;
}
