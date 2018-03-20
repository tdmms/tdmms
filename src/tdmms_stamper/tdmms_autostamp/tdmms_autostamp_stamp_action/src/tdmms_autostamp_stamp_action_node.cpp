/////////////////////////////////////
//  Copyright 2016.08 by S. Masubuchi
//  2DMMS Stamp Action Node
/////////////////////////////////////
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_stamp_action/StampAction.h>
#include <geometry_msgs/Pose.h>
#include <stamper_sample_z/velocity.h>
#include <geometry_msgs/Point.h>
#include <tdmms_autostamp_camera_streamer/CaptureImageTimestamp.h>
#include <string>

class autostamp_stamp_class {
 protected:
  ros::NodeHandle node_;
  actionlib::SimpleActionServer<tdmms_autostamp_stamp_action::StampAction> as_;
  std::string action_name_;
  tdmms_autostamp_stamp_action::StampFeedback feedback_;
  tdmms_autostamp_stamp_action::StampResult result_;
  ros::Publisher stamper_nikon_cmd_z_abs;
  ros::Subscriber stamper_currload_subscriber;
  ros::Publisher stamper_sample_z_cmd_stp;
  ros::Publisher stamper_sample_z_cmd_vel;
  ros::Publisher stamper_sample_z_cmd_abs;
  ros::ServiceClient stamper_sample_z_wait_for_stop_client;
  ros::ServiceClient autostamp_camera_streamer_capture_timestamp;
  int currentLoad;
  enum E_currentState {
    Stop,
    Start,
    Up,
    Contact,
    Down
  } currentState;

 public:
  autostamp_stamp_class(std::string name)
      : as_(node_, name,
            boost::bind(&autostamp_stamp_class::executeStamp, this, _1), false),
        action_name_(name) {
    currentLoad = 0;

    /////////////////////////////
    /// Initialize Publishers
    ////////////////////////////
    stamper_nikon_cmd_z_abs =
        node_.advertise<std_msgs::UInt32>("/stamper_nikon_master/cmd_z_abs", 1);
    stamper_sample_z_cmd_stp = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_z_master/cmd_stp_move", 1);
    stamper_sample_z_cmd_vel = node_.advertise<stamper_sample_z::velocity>(
        "/stamper_sample_z_master/cmd_vel", 1);
    stamper_sample_z_cmd_abs = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_z_master/cmd_abs_move", 1);

    ////////////////////////////////
    //// Initialize Subscribers
    ////////////////////////////////
    stamper_currload_subscriber =
        node_.subscribe("/stamper_loadcell/value", 1,
                        &autostamp_stamp_class::currload_Callback, this);

    ////////////////////////////////
    //// Initialize Service Clients
    ////////////////////////////////
    stamper_sample_z_wait_for_stop_client =
        node_.serviceClient<std_srvs::Empty>(
            "/stamper_sample_z_master/wait_for_stop");
    autostamp_camera_streamer_capture_timestamp = node_.serviceClient<
        tdmms_autostamp_camera_streamer::CaptureImageTimestamp>(
        "/autostamp_camera_streamer/capture_timestamp");

    as_.start();
  }

  ~autostamp_stamp_class(void) {}

  void currload_Callback(const std_msgs::Float32 &load) {
    currentLoad = (load.data * 1000);
  }

  void executeStamp(
      const tdmms_autostamp_stamp_action::StampGoalConstPtr &goalparam) {
    // as_.acceptNewGoal();
    std_msgs::UInt32 z_stage_pos;
    stamper_nikon_cmd_z_abs.publish(goalparam->om_position_at_stamp);

    stamper_sample_z::velocity vel_sample_z;
    vel_sample_z.velocity_low = 1000;
    vel_sample_z.velocity_high = 15000;
    vel_sample_z.accl = 100;
    stamper_sample_z_cmd_vel.publish(vel_sample_z);

    ros::Duration(0.5).sleep();
    stamper_sample_z_cmd_abs.publish(goalparam->z_stage_position_at_stamp);
    ros::Duration(0.1).sleep();
    std_srvs::Empty emp_srv;
    stamper_sample_z_wait_for_stop_client.call(emp_srv);

    ros::Duration(0.5).sleep();
    ////////////////////////////////
    //// Capture Image
    /////////////////////////////////
    tdmms_autostamp_camera_streamer::CaptureImageTimestamp cap_timestamp;
    cap_timestamp.request.folder = goalparam->folder.data.c_str();
    autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);

    geometry_msgs::Point pnt;
    currentState = Up;
    ros::Rate loop_rate_up(goalparam->up_speed.data);
    while (1) {
      ros::spinOnce();
      if (!as_.isPreemptRequested() && ros::ok()) {
        switch (currentState) {
          case Up:
            pnt.x = 1;
            pnt.y = 0;
            stamper_sample_z_cmd_stp.publish(pnt);
            loop_rate_up.sleep();
            if (abs(currentLoad) > goalparam->stamp_load.data) {
              currentState = Contact;
            }
            ROS_INFO("StampSequcne UP, CurrentLoad:%d mN", currentLoad);
            break;
          case Contact:
            ////////////////////////////////
            //// Capture Image
            /////////////////////////////////
            cap_timestamp.request.folder = goalparam->folder.data.c_str();
            autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);

            ROS_INFO("StampSequence Contact, Wait for %d sec",
                     goalparam->stamp_duration.data);
            ros::Duration(goalparam->stamp_duration.data).sleep();

            ////////////////////////////////
            //// Capture Image
            /////////////////////////////////
            cap_timestamp.request.folder = goalparam->folder.data.c_str();
            autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);
            currentState = Down;
            break;
          case Stop:
            break;
          case Down:
            ROS_INFO("StampSequence Down");
            /////////////////////////////
            /// Set Velocity (Slow)
            ////////////////////////////
            stamper_sample_z::velocity vel;
            vel.velocity_low = 100;
            vel.velocity_high = static_cast<int>(goalparam->down_speed.data);
            vel.accl = 100;
            stamper_sample_z_cmd_vel.publish(vel);
            ros::Duration(0.5).sleep();

            //////////////////////////////
            //// Step Down (Slightly)
            //////////////////////////////
            pnt.x = -2000;
            pnt.y = 0;
            stamper_sample_z_cmd_stp.publish(pnt);
            ros::Duration(0.1).sleep();
            stamper_sample_z_wait_for_stop_client.call(emp_srv);
            ros::Duration(0.1).sleep();

            ////////////////////////////////
            //// Capture Image
            /////////////////////////////////
            cap_timestamp.request.folder = goalparam->folder.data.c_str();
            autostamp_camera_streamer_capture_timestamp.call(cap_timestamp);

            /////////////////////////////
            /// Set Velocity (High)
            ////////////////////////////
            vel_sample_z.velocity_low = 1000;
            vel_sample_z.velocity_high = 15000;
            vel_sample_z.accl = 100;
            stamper_sample_z_cmd_vel.publish(vel_sample_z);
            ros::Duration(0.1).sleep();

            //////////////////////////////
            //// Step Down (To the initial position)
            //////////////////////////////
            pnt.x = 585000;
            pnt.y = 0;
            stamper_sample_z_cmd_abs.publish(pnt);
            ros::Duration(0.1).sleep();
            stamper_sample_z_wait_for_stop_client.call(emp_srv);

            currentState = Stop;
            as_.setSucceeded();
            return;
        }
      } else {
        as_.setPreempted();
      }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tdmms_autostamp_stamp_action");

  autostamp_stamp_class stamp_action(ros::this_node::getName());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  ros::spin();

  return 0;
}
