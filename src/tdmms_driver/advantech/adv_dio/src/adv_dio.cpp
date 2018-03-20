//////////////////////////////////////
// Copyright 2016 by S. Masubuchi
//////////////////////////////////////

#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <comedilib.h>
#include <std_msgs/UInt16.h>
#include <adv_dio/dio_write.h>

comedi_t *it;

bool dio_write(
    adv_dio::dio_write::Request &req,
    adv_dio::dio_write::Response &resp) {
  const unsigned int subdev = 3;
  for (unsigned int chan = 0 ; chan < 16; chan++) {
    int retval;
    unsigned int bit = 0x01 & (req.value >> chan);
    retval = comedi_dio_write(it, subdev, chan, bit);

    if (retval < 0) {
      ROS_ERROR("Comedi DIO Write Error");
      resp.success = false;
      return false;
    }
  }
  resp.success = true;
  return true;
}

int main(int argc, char *argv[]) {
  const unsigned int subdev = 1;
  ros::Publisher status_pub;
  ros::init(argc, argv, "adv_dio");
  ros::NodeHandle node;
  ros::ServiceServer service;

  status_pub = node.advertise<std_msgs::UInt16>(
      "/adv_dio/status", 100);
  service = node.advertiseService("/adv_dio/write", dio_write);

  it = comedi_open("/dev/comedi0");
  if (it == NULL) {
    ROS_ERROR("Comedi Open Error");
    return -1;
  }

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    std_msgs::UInt16 status;
    status.data = 0;
    for (unsigned int chan = 0 ; chan < 16; chan ++) {
      int retval;
      unsigned int bit = 0;
      retval = comedi_dio_read(it, subdev, chan, &bit);
      if (retval < 0) {
        ROS_ERROR("Comedi DIO Read Error");
        comedi_close(it);
        return -1;
      }
      if(bit == 1) bit = 0;
      else if(bit == 0) bit = 1;
      status.data |= (bit << chan);
    }
    status_pub.publish(status);
    ros::spinOnce();
    loop_rate.sleep();
  }

  comedi_close(it);
  return 0;
}
