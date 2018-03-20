//////////////////////////////////////
// Copyright 2016 by S. Masubuchi
//////////////////////////////////////

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>

#include <tdmms_autostamp_camera_streamer/CaptureImage.h>
#include <tdmms_autostamp_camera_streamer/CaptureImageTimestamp.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

HalconCpp::HObject ho_Image;

bool capture_image(
    tdmms_autostamp_camera_streamer::CaptureImage::Request &req,
    tdmms_autostamp_camera_streamer::CaptureImage::Response &resp) {
  try {
    WriteImage(ho_Image, "jpg", 0, req.filename.c_str());
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    ROS_ERROR("Could not Save Image");
    resp.success = false;
    return false;
  }

  resp.success = true;
  return true;
}

bool capture_image_timestamp(
    tdmms_autostamp_camera_streamer::CaptureImageTimestamp::Request &req,
    tdmms_autostamp_camera_streamer::CaptureImageTimestamp::Response &resp) {
  char filename[4096];

  try {
    struct timeval myTime;
    struct tm *time_st;
    char buffer[256];
    gettimeofday(&myTime, NULL);
    time_st = localtime(&myTime.tv_sec);

    sprintf(buffer, "%d-%02d-%02d-%02d-%02d-%02d.%06d",
             time_st->tm_year + 1900, time_st->tm_mon + 1, time_st->tm_mday,
             time_st->tm_hour, time_st->tm_min, time_st->tm_sec,
              (int)myTime.tv_usec);

snprintf(filename, sizeof(filename) ,"%s/%s.jpg", req.folder.c_str(),
             buffer);
    WriteImage(ho_Image, "jpg", 0, filename);
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    ROS_ERROR("Could not Save Image");
    resp.success = false;
    return false;
  }

  resp.success = true;
  return true;
}

int main(int argc, char *argv[]) {
  ros::Publisher image_pub_main;
  ros::init(argc, argv, "autostamp_camera_streamer");
  ros::NodeHandle node;
  ros::ServiceServer service;
  ros::ServiceServer service_timestamp;
  sensor_msgs::Image Image;
  HalconCpp::HTuple hv_AcqHandle;

  image_pub_main = node.advertise<sensor_msgs::Image>(
      "/autostamp_camera_streamer/image_main", 100);

  service = node.advertiseService("/autostamp_camera_streamer/capture",
                                  capture_image);
  service_timestamp = node.advertiseService(
      "/autostamp_camera_streamer/capture_timestamp", capture_image_timestamp);

  ros::Rate loop_rate(30);

  try {
    HalconCpp::OpenFramegrabber("Video4Linux2", 1, 1, 0, 0, 0, 0, "progressive",
                                8, "default", -1, "false", "auto", "video0", 0,
                                -1, &hv_AcqHandle);
    HalconCpp::GrabImageStart(hv_AcqHandle, -1);
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    ROS_ERROR("Could not Open Framegrabber");
    exit(-1);
  }
  ROS_INFO("Frage Grabber Opened, Start Streaming Image..");
  while (ros::ok()) {
    HalconCpp::GrabImageAsync(&ho_Image, hv_AcqHandle, -1);
    halcon_bridge::toImageMsg(ho_Image, Image);
    image_pub_main.publish(Image);
    ros::spinOnce();
    loop_rate.sleep();
  }
  HalconCpp::CloseFramegrabber(hv_AcqHandle);

  return 0;
}
