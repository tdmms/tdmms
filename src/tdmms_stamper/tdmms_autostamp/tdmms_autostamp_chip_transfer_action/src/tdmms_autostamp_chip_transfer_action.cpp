/////////////////////////////////////////////
// Copyright 2016 by S. Masubuchi
/////////////////////////////////////////////

#include <arpa/inet.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_chip_transfer_action/TransferChipAction.h>
#include <stamper_yamaha/PointNo.h>
#include <stamper_yamaha/AbsMove.h>
#include <stamper_yamaha/PalletPos.h>
#include <keyence_master/Status.h>
#include <stamper_sample_z/GetCurrentPos.h>

class tdmms_autostamp_chip_transfer_action_class {
 protected:
  ros::NodeHandle node_;
  ros::Publisher transfer_valve_open_publisher;
  ros::Publisher transfer_valve_close_publisher;
  ros::ServiceClient yamaha_service_ptp;
  ros::ServiceClient yamaha_service_abs;
  ros::ServiceClient yamaha_service_linear;
  ros::ServiceClient keyence_status_client;
  ros::ServiceClient stamper_sample_z_get_current_pos_client;
  actionlib::SimpleActionServer<
      tdmms_autostamp_chip_transfer_action::TransferChipAction> as_;
  std::string action_name_;
  tdmms_autostamp_chip_transfer_action::TransferChipFeedback feedback_;
  tdmms_autostamp_chip_transfer_action::TransferChipResult result_;

 public:
  tdmms_autostamp_chip_transfer_action_class(std::string name)
      : as_(node_, name,
            boost::bind(&tdmms_autostamp_chip_transfer_action_class::
                             executeTransferChip,
                        this, _1),
            false),
        action_name_(name) {
    transfer_valve_open_publisher =
        node_.advertise<std_msgs::Empty>("/transfer_valve/cmd_open", 1);
    transfer_valve_close_publisher =
        node_.advertise<std_msgs::Empty>("/transfer_valve/cmd_close", 1);
    yamaha_service_ptp = node_.serviceClient<stamper_yamaha::PointNo>(
        "/stamper_yamaha_master/cmd_ptp_move");
    yamaha_service_abs = node_.serviceClient<stamper_yamaha::AbsMove>(
        "/stamper_yamaha_master/cmd_abs_move_linear");
    yamaha_service_linear = node_.serviceClient<stamper_yamaha::PointNo>(
        "/stamper_yamaha_master/cmd_linear_move");
    keyence_status_client = node_.serviceClient<keyence_master::Status>(
        "/keyence_master/get_status");
    stamper_sample_z_get_current_pos_client =
        node_.serviceClient<stamper_sample_z::GetCurrentPos>(
            "/stamper_sample_z_master/get_currpos");
    as_.start();
  }

  ~tdmms_autostamp_chip_transfer_action_class(void) {}

  bool checkPreempt() {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      return true;
    }
    return false;
  }

  void executeTransferChip(
      const tdmms_autostamp_chip_transfer_action::TransferChipGoalConstPtr &
          goalparam) {
    stamper_yamaha::PointNo point;
    stamper_yamaha::PalletPos pallet;
    std_msgs::Empty emp;

    // Check Pocket No within a range
    if (goalparam->pocketNo.data <= 0 || goalparam->pocketNo.data > 36) {
      ROS_ERROR("pocketNo Out of range");
      as_.setAborted(result_);
      return;
    }
    // Check Pallet No within a range
    if (goalparam->palletNo.data < 0 || goalparam->palletNo.data > 2) {
      ROS_ERROR("palletNo Out of Range");
      as_.setAborted(result_);
      return;
    }

    // Check OM, Mask Position
    keyence_master::Status stat;

    if (!keyence_status_client.call(stat)) {
      ROS_ERROR("Keyence PLC Communication Error");
      as_.setAborted(result_);
      return;
    }

    if (!stat.response.mask_back) {
      ROS_ERROR("Tried to Transfer Chip while Mask is at Alignment Position");
      as_.setAborted(result_);
      return;
    }

    if (!stat.response.om_back) {
      ROS_ERROR(
          "Tried to Transfer Chip while Optical Microscpoe is at Alignment "
          "Position");
      as_.setAborted(result_);
      return;
    }

    stamper_sample_z::GetCurrentPos pos;
    if (!stamper_sample_z_get_current_pos_client.call(pos)) {
      ROS_ERROR("Stamper Z stage Communication Error");
      as_.setAborted(result_);
      return;
    }

    if (!(pos.response.pos < 5100)) {
      ROS_ERROR("Z stage not located at home position");
      as_.setAborted(result_);
      return;
    }

    stamper_yamaha::AbsMove moveto;
    const double stage_height = 80.6;
    moveto.request.targetPose.position.x = 589.793;
    moveto.request.targetPose.position.y = 351.492;
    moveto.request.targetPose.position.z = 0;

    const int vel_high = 20;
    const int vel_low = 2;
    const int vel_linear = 100;

    
    if (goalparam->rotangle_deg.data < -180)
      moveto.request.targetPose.orientation.w =
          goalparam->rotangle_deg.data + 360;
    else
      moveto.request.targetPose.orientation.w = goalparam->rotangle_deg.data;

    if (goalparam->isLoading.data) {
      point.request.pointno.data = 0;  // Origin
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 1;
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 2;
      point.request.velocity.data = vel_linear;
      yamaha_service_linear.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 4000 + (1000 * goalparam->palletNo.data) +
                                   goalparam->pocketNo.data + 36;
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data =
          4000 + (1000 * goalparam->palletNo.data) + goalparam->pocketNo.data;
      point.request.velocity.data = vel_low;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      transfer_valve_open_publisher.publish(emp);
      ros::Duration(1).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 4000 + (1000 * goalparam->palletNo.data) +
                                   goalparam->pocketNo.data + 36;
      point.request.velocity.data = vel_low;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 2;
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 1;
      point.request.velocity.data = vel_linear;
      yamaha_service_linear.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.z = 0.00;
      moveto.request.velocity.data = vel_high;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      //  Mask Aligner Stage
      moveto.request.targetPose.position.z = stage_height -10; 
      moveto.request.velocity.data = vel_high;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      //  Mask Aligner Stage
      moveto.request.targetPose.position.z = stage_height;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      transfer_valve_close_publisher.publish(emp);
      ros::Duration(10).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      //  Mask Aligner Stage
      moveto.request.targetPose.position.z = stage_height -10; 
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.z = 0.00;
      moveto.request.velocity.data = vel_high;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 0;  // Origin
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

    } else {
      point.request.pointno.data = 0;  // Origin
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.z = 0.00;
      moveto.request.velocity.data = vel_high;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.z = stage_height -10; 
      moveto.request.velocity.data = vel_high;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.z = stage_height;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      transfer_valve_open_publisher.publish(emp);
      ros::Duration(1).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.x += 1.5;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.x -= 3.0;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.y += 1.5;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.y -= 3.0;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.y += 1.5;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.x += 1.5;
      moveto.request.targetPose.position.y += 1.5;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.x -= 3.0;
      moveto.request.targetPose.position.y -= 3.0;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.x += 1.5;
      moveto.request.targetPose.position.y -= 1.5;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.x -= 3.0;
      moveto.request.targetPose.position.y += 3.0;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.x += 1.5;
      moveto.request.targetPose.position.y -= 1.5;
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      ros::Duration(0.5).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.z = stage_height -10; 
      moveto.request.velocity.data = vel_low;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      moveto.request.targetPose.position.z = 0.00;
      moveto.request.velocity.data = vel_high;
      yamaha_service_abs.call(moveto);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 0;
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 1;
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 2;
      point.request.velocity.data = vel_linear;
      yamaha_service_linear.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 4000 + (1000 * goalparam->palletNo.data) +
                                   goalparam->pocketNo.data + 36;
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data =
          4000 + (1000 * goalparam->palletNo.data) + goalparam->pocketNo.data;
      point.request.velocity.data = vel_low;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      transfer_valve_close_publisher.publish(emp);
      ros::Duration(6).sleep();
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.velocity.data = vel_low;
      point.request.pointno.data = 4000 + (1000 * goalparam->palletNo.data) +
                                   goalparam->pocketNo.data + 36;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 2;
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 1;
      point.request.velocity.data = vel_linear;
      yamaha_service_linear.call(point);
      if (checkPreempt()) {as_.setPreempted(); return;}

      point.request.pointno.data = 0;  // Origin
      point.request.velocity.data = vel_high;
      yamaha_service_ptp.call(point);
    }

    result_.Position_Result = feedback_.Position_Current;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tdmms_autostamp_chip_transfer_action");

  tdmms_autostamp_chip_transfer_action_class
      tdmms_autostamp_chip_transfer_action(ros::this_node::getName());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  ros::spin();

  return 0;
}
