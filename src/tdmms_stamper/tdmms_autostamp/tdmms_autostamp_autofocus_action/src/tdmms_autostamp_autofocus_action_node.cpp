// Copyright 2016 by S. Masubuchi

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <stamper_sample_z/velocity.h>
#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_autofocus_action/AutoFocusAction.h>
#include <stamper_sample_xy/velocity.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>

#include <string>

#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"

using namespace HalconCpp;
class autostamp_autofocus_class {
 protected:
  ros::NodeHandle node_;
  ros::Publisher stamper_nikon_cmd_z_abs;
  ros::Publisher stamper_nikon_revolv_set;
  ros::Subscriber image_subscriber_main;
  ros::Publisher stamper_sample_z_cmd_stp;
  ros::Publisher stamper_sample_z_cmd_vel;
  ros::Publisher stamper_sample_z_cmd_abs;
  ros::Publisher stamper_nic_100a_cmd_inten;
  ros::ServiceClient stamper_sample_z_wait_for_stop_client;

  actionlib::SimpleActionServer<
      tdmms_autostamp_autofocus_action::AutoFocusAction> as_;
  std::string action_name_;
  tdmms_autostamp_autofocus_action::AutoFocusFeedback feedback_;
  tdmms_autostamp_autofocus_action::AutoFocusResult result_;
  HObject ho_Image, ho_Image_buf;
  HTuple hv_WindowHandle_main, hv_WindowHandle_sub, hv_DeviceIdentifiers;

 public:
  autostamp_autofocus_class(std::string name)
      : as_(node_, name,
            boost::bind(&autostamp_autofocus_class::executeAutoFocus, this, _1),
            false),
        action_name_(name) {
    stamper_nikon_revolv_set = node_.advertise<std_msgs::UInt8>(
        "/stamper_nikon_master/cmd_revolv_set", 1);
    node_.advertise<std_msgs::UInt8>("/stamper_nic_100a/cmd_inten", 1);
    stamper_nikon_cmd_z_abs =
        node_.advertise<std_msgs::UInt32>("/stamper_nikon_master/cmd_z_abs", 1);
    stamper_sample_z_cmd_stp = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_z_master/cmd_stp_move", 1);
    stamper_sample_z_cmd_vel = node_.advertise<stamper_sample_z::velocity>(
        "/stamper_sample_z_master/cmd_vel", 1);
    stamper_sample_z_cmd_abs = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_z_master/cmd_abs_move", 1);
    stamper_nic_100a_cmd_inten =
        node_.advertise<std_msgs::UInt8>("/stamper_nic_100a/cmd_inten", 1);
    stamper_sample_z_wait_for_stop_client =
        node_.serviceClient<std_srvs::Empty>(
            "/stamper_sample_z_master/wait_for_stop");
    

    as_.start();
  }

  ~autostamp_autofocus_class(void) {}

  void imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image) {
    halcon_bridge::toHObject(Image, &ho_Image_buf);
    ho_Image = ho_Image_buf;
  }

  void executeAutoFocus(
      const tdmms_autostamp_autofocus_action::AutoFocusGoalConstPtr &
          goalparam) {
    //    as_.acceptNewGoal();

    std_srvs::Empty emp_srv;
    geometry_msgs::Point pnt;
    pnt.x = 585000;
    pnt.y = 0;
    stamper_sample_z_cmd_abs.publish(pnt);
    ros::Duration(0.1).sleep();
    stamper_sample_z_wait_for_stop_client.call(emp_srv);

    image_subscriber_main =
        node_.subscribe("/autostamp_camera_streamer/image_main", 1,
                        &autostamp_autofocus_class::imageStreamCallback, this);

    ros::Duration(1).sleep();
    ////////////////////////////////////////
    /// Set Objective Lens Tartlet to 5X
    ////////////////////////////////////////
    std_msgs::UInt8 pos;
    pos.data = 1;
    stamper_nikon_revolv_set.publish(pos);
    ros::Duration(0.1).sleep();

    std_msgs::UInt8 light_inten;
    light_inten.data = 5;
    stamper_nic_100a_cmd_inten.publish(light_inten);
    ros::Duration(1).sleep();

    ///////////////////////////////////////
    /// Set Z stage of Optical Microscope
    //////////////////////////////////////
    std_msgs::UInt32 z_stage_pos;
    z_stage_pos.data = goalparam->start.data;
    stamper_nikon_cmd_z_abs.publish(z_stage_pos);
    geometry_msgs::Point point;

    // Local iconic variables
    HObject ho_Highpass, ho_Rect;
    HObject ho_ImageGray;
    // Local control variables
    HTuple hv_Index, hv_Width, hv_Height, hv_Mean;
    HTuple hv_Deviation, hv_deviation_tuple, hv_max_dev, hv_ind;
    HTuple hv_Deviation_prev;
    ros::Duration(2).sleep();
    int i;
    for (i = 1; i <= goalparam->stepno.data; i += 1) {
      try {
        z_stage_pos.data =
            goalparam->start.data + (i - 1) * goalparam->delta_z.data;
        stamper_nikon_cmd_z_abs.publish(z_stage_pos);
        ros::Duration(0.3).sleep();

        Rgb1ToGray(ho_Image, &ho_ImageGray);
        HighpassImage(ho_ImageGray, &ho_Highpass, 9, 9);
        GetImageSize(ho_Highpass, &hv_Width, &hv_Height);
        GenRectangle1(&ho_Rect, 0, 0, hv_Height, hv_Width);
        Intensity(ho_Rect, ho_Highpass, &hv_Mean, &hv_Deviation);
        hv_deviation_tuple[i] = hv_Deviation;

        if (as_.isPreemptRequested() || !ros::ok()) {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          return;
        }
        ROS_INFO("deviation: %f", static_cast<double>(hv_deviation_tuple[i]));
      }
      catch (HalconCpp::HException &HDevExpDefaultException) {
        continue;
      }
    }
    ros::Duration(0.5).sleep();

    for (i = 1; i <= goalparam->stepno.data - 1; i++) {
      if (hv_deviation_tuple[i + 1] - hv_deviation_tuple[i] > 1.0) {
        hv_deviation_tuple[i + 1] = 0.0;
        i++;
      }
    }
    TupleMax(hv_deviation_tuple, &hv_max_dev);
    TupleFind(hv_deviation_tuple, hv_max_dev, &hv_ind);
    z_stage_pos.data = (goalparam->start.data) +
                       (static_cast<int>(hv_ind) - 1) * goalparam->delta_z.data;
    stamper_nikon_cmd_z_abs.publish(z_stage_pos);
    image_subscriber_main.shutdown();
    result_.focuspos.data = z_stage_pos.data;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tdmms_autostamp_autofocus_action");
  autostamp_autofocus_class autofocus(ros::this_node::getName());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  ros::spin();
  return 0;
}
