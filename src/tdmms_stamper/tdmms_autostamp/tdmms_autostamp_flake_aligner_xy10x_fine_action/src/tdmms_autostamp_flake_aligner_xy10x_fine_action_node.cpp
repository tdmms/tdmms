////////////////////////////////////////////
// Copyright 2016 by S. Masubuchi
///////////////////////////////////////////
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <stdlib.h>
#include <stdio.h>

#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_flake_aligner_xy10x_fine_action/AlignFlakeXY10x_FineAction.h>
#include <geometry_msgs/Pose.h>
#include <adm2/velocity.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>

#include <string>

#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"

using namespace HalconCpp;
class autostamp_flake_aligner_xy10x_fine_class {
 protected:
  ros::NodeHandle node_;
  ros::Subscriber image_subscriber_main;
  actionlib::SimpleActionServer<
      tdmms_autostamp_flake_aligner_xy10x_fine_action::
          AlignFlakeXY10x_FineAction> as_;
  std::string action_name_;
  tdmms_autostamp_flake_aligner_xy10x_fine_action::AlignFlakeXY10x_FineFeedback
      feedback_;
  tdmms_autostamp_flake_aligner_xy10x_fine_action::AlignFlakeXY10x_FineResult
      result_;
  HTuple hv_AcqHandle, hv_DeviceHandle, hv_Index, hv_DeviceName;
  HTuple hv_DeviceVendor;
  HObject ho_Image, ho_Image_buf;
  HTuple hv_WindowHandle_main, hv_WindowHandle_sub, hv_DeviceIdentifiers;
  ros::Publisher stamper_sample_xy_cmd_vel;
  ros::Publisher stamper_sample_xy_cmd_jog;
  ros::Publisher stamper_sample_xy_cmd_abs;
  ros::Publisher stamper_sample_xy_cmd_stop;
  ros::Publisher stamper_sample_xy_cmd_stp;
  ros::ServiceClient stamper_sample_xy_wait_for_stop_client;
  ros::Publisher stamper_sample_theta_cmd_vel;
  ros::Publisher stamper_sample_theta_cmd_jog;
  ros::Publisher stamper_sample_theta_cmd_abs;
  ros::Publisher stamper_sample_theta_cmd_stop;
  ros::Publisher stamper_nikon_revolv_set;
  ros::Publisher stamper_nikon_cmd_z_abs;
  ros::Publisher stamper_nic_100a_cmd_inten;
  double om_pixel_to_step_factor;
  double cyclemove_deltay_stp_10x;
  double cyclemove_deltax_stp_10x;
  double alignment_thresh_stp;

 public:
  autostamp_flake_aligner_xy10x_fine_class(std::string name)
      : as_(node_, name,
            boost::bind(
                &autostamp_flake_aligner_xy10x_fine_class::executeAlignFlake,
                this, _1),
            false),
        action_name_(name) {
    /*
    QueryAvailableComputeDevices(&hv_DeviceIdentifiers);
    {
      HalconCpp::HTuple end_val3 = (hv_DeviceIdentifiers.TupleLength()) - 1;
      HalconCpp::HTuple step_val3 = 1;
      for (hv_Index = 0; hv_Index.Continue(end_val3, step_val3);
           hv_Index += step_val3) {
        GetComputeDeviceInfo(HTuple(hv_DeviceIdentifiers[hv_Index]), "name",
                             &hv_DeviceName);
        GetComputeDeviceInfo(HTuple(hv_DeviceIdentifiers[hv_Index]), "vendor",
                             &hv_DeviceVendor);
        OpenComputeDevice(HTuple(hv_DeviceIdentifiers[hv_Index]),
                          &hv_DeviceHandle);
      }
    }
    ActivateComputeDevice(hv_DeviceHandle);
    */
    stamper_sample_xy_cmd_vel =
        node_.advertise<adm2::velocity>("/stamper_sample_xy_master/cmd_vel", 1);
    stamper_sample_xy_cmd_jog = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_jog_move", 1);
    stamper_sample_xy_cmd_abs = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_abs_move", 1);
    stamper_sample_xy_cmd_stop = node_.advertise<std_msgs::Empty>(
        "/stamper_sample_xy_master/cmd_stop", 1);
    stamper_sample_xy_cmd_stp = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_stp_move", 1);
    stamper_sample_xy_wait_for_stop_client =
        node_.serviceClient<std_srvs::Empty>(
            "/stamper_sample_xy_master/wait_for_stop");

    stamper_sample_theta_cmd_vel = node_.advertise<adm2::velocity>(
        "/stamper_sample_theta_master/cmd_vel", 1);
    stamper_sample_theta_cmd_jog = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_theta_master/cmd_jog_move", 1);
    stamper_sample_theta_cmd_abs = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_theta_master/cmd_abs_move", 1);
    stamper_sample_theta_cmd_stop = node_.advertise<std_msgs::Empty>(
        "/stamper_sample_theta_master/cmd_stop", 1);

    stamper_nikon_revolv_set = node_.advertise<std_msgs::UInt8>(
        "/stamper_nikon_master/cmd_revolv_set", 1);

    stamper_nic_100a_cmd_inten =
        node_.advertise<std_msgs::UInt8>("/stamper_nic_100a/cmd_inten", 1);
    stamper_nikon_cmd_z_abs =
        node_.advertise<std_msgs::UInt32>("/stamper_nikon_master/cmd_z_abs", 1);

    std::string parameter_name;
    parameter_name = "/tdmms_stamper_autostamp/om_pixel_to_step_factor_10x";
    if (!node_.getParam(parameter_name, om_pixel_to_step_factor)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }
    parameter_name = "/tdmms_stamper_autostamp/cyclemove_deltax_stp_10x";
    if (!node_.getParam(parameter_name, cyclemove_deltax_stp_10x)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }
    parameter_name = "/tdmms_stamper_autostamp/cyclemove_deltay_stp_10x";
    if (!node_.getParam(parameter_name, cyclemove_deltay_stp_10x)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }
    parameter_name = "/tdmms_stamper_autostamp/alignment_thresh_stp_10x";
    if (!node_.getParam(parameter_name, alignment_thresh_stp)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }

    as_.start();
  }

  ~autostamp_flake_aligner_xy10x_fine_class(void) {}

  void imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image) {
    halcon_bridge::toHObject(Image, &ho_Image_buf);
    ho_Image = ho_Image_buf;
  }

  void executeAlignFlake(const tdmms_autostamp_flake_aligner_xy10x_fine_action::
                             AlignFlakeXY10x_FineGoalConstPtr &goalparam) {
    bool success = true;

    std::string initfname_small;
    initfname_small = getenv("HOME");
    initfname_small = initfname_small + "/img744x480.jpg";
    ReadImage(&ho_Image, initfname_small.c_str());
    HalconCpp::SetWindowAttr("background_color", "black");
    HalconCpp::OpenWindow(0, 744 * 1.5, 744 * 1.5, 480 * 1.5, 0, "", "",
                          &hv_WindowHandle_main);
    HalconCpp::HDevWindowStack::Push(hv_WindowHandle_main);

    std::string initfname_large;
    initfname_large = getenv("HOME");
    initfname_large = initfname_large + "/img2040x1086.jpg";
    ReadImage(&ho_Image, initfname_large.c_str());
    HalconCpp::OpenWindow(0, 2560, 744 * 1.5, 480 * 1.5, 0, "", "",
                          &hv_WindowHandle_sub);
    HalconCpp::HDevWindowStack::Push(hv_WindowHandle_sub);
    halcon_bridge::set_display_font(hv_WindowHandle_main, 16, "mono", "true",
                                    "false");
    halcon_bridge::set_display_font(hv_WindowHandle_sub, 16, "mono", "true",
                                    "false");

    image_subscriber_main = node_.subscribe(
        "/autostamp_camera_streamer/image_main", 100,
        &autostamp_flake_aligner_xy10x_fine_class::imageStreamCallback, this);

    ros::Duration(1).sleep();

    ////////////////////////////////////////
    /// Set Objective Lens Tartlet to 10X
    ////////////////////////////////////////
    std_msgs::UInt8 pos;
    pos.data = 2;
    stamper_nikon_revolv_set.publish(pos);
    ros::Duration(1).sleep();

    //////////////////////////////////////
    //// Set Objective Light Intensity to 5
    ///////////////////////////////////////
    std_msgs::UInt8 light_inten;
    light_inten.data = 5;
    stamper_nic_100a_cmd_inten.publish(light_inten);

    ///////////////////////////////////////
    ////// Set Z stage of objective lens
    //////////////////////////////////////
    std_msgs::UInt32 focuspos_offset;
    focuspos_offset = goalparam->focuspos;
    focuspos_offset.data = focuspos_offset.data - 450;
    stamper_nikon_cmd_z_abs.publish(focuspos_offset);

    ros::Duration(2).sleep();

    // Local iconic variables
    HObject ho_ModelRegion, ho_TemplateImage;
    HObject ho_ModelContours, ho_Image_Match;

    // Local control variables
    HTuple hv_DeviceIdentifiers, hv_Index, hv_DeviceName;
    HTuple hv_DeviceVendor, hv_DeviceHandle, hv_Protocol, hv_Timeout;
    HTuple hv_Socket, hv_Address, hv_To, hv_pix_to_step_factor;
    HTuple hv_WindowHandle2;
    HTuple hv_f0, hv_f1, hv_i, hv_fib, hv_cycle_deltax_stp;
    HTuple hv_cycle_deltay_stp, hv_count, hv_k, hv_j, hv_uzumaki_vector_x;
    HTuple hv_uzumaki_vector_y, hv_cycle_deltax_stp_buf,
        hv_cycle_deltay_stp_buf;
    HTuple hv_cycle, hv_ModelId, hv_Width, hv_Height, hv_Seconds1;
    HTuple hv_ModelRow, hv_ModelColumn, hv_ModelAngle, hv_ModelScale;
    HTuple hv_ModelScore, hv_Seconds2;
    HTuple hv_deltax, hv_deltay, hv_movevector_x, hv_movevector_y;
    HTuple hv_movevector_x_c, hv_movevector_y_c;

    geometry_msgs::Point point;
    std_srvs::Empty emp_srv;

    hv_pix_to_step_factor = om_pixel_to_step_factor / 5;
    SetWindowAttr("background_color", "black");
    ReadImage(&ho_Image_Match, HTuple(goalparam->filename_target.data.c_str()));

    //////////////////////////////////
    /// Create Displacement Vectors
    ///////////////////////////////////
    hv_f0 = 0;
    hv_f1 = 1;
    for (hv_i = 0; hv_i <= 100; hv_i += 1) {
      hv_fib[hv_i] = hv_f0 + hv_f1;
      hv_f0 = hv_f1;
      hv_f1 = ((const HTuple &)hv_fib)[hv_i];
    }
    hv_cycle_deltax_stp = cyclemove_deltax_stp_10x;
    hv_cycle_deltay_stp = cyclemove_deltay_stp_10x;

    hv_count = 0;
    for (hv_i = 0; hv_i <= 20; hv_i += 1) {
      for (hv_k = 0; hv_k <= 1; hv_k += 1) {
        {
          HTuple end_val35 = HTuple(hv_fib[hv_i]) - 1;
          HTuple step_val35 = 1;
          for (hv_j = 0; hv_j.Continue(end_val35, step_val35);
               hv_j += step_val35) {
            hv_uzumaki_vector_x[hv_count] = hv_cycle_deltax_stp;
            hv_uzumaki_vector_y[hv_count] = hv_cycle_deltay_stp;
            hv_count += 1;
          }
        }
        hv_cycle_deltax_stp_buf = -hv_cycle_deltay_stp;
        hv_cycle_deltay_stp_buf = hv_cycle_deltax_stp;
        hv_cycle_deltax_stp = hv_cycle_deltax_stp_buf;
        hv_cycle_deltay_stp = hv_cycle_deltay_stp_buf;
      }
    }

    hv_cycle = 0;
    while (true) {
      HDevWindowStack::SetActive(hv_WindowHandle_main);
      if (HDevWindowStack::IsOpen())
        DispObj(ho_Image, HDevWindowStack::GetActive());

      HDevWindowStack::SetActive(hv_WindowHandle_sub);
      SetSystem("border_shape_models", "true");
      GenRectangle1(&ho_ModelRegion, 0, 0, 475.5, 736.5);
      ReduceDomain(ho_Image, ho_ModelRegion, &ho_TemplateImage);
      try {
        CreateScaledShapeModel(
            ho_TemplateImage, 4, HTuple(-2).TupleRad(), HTuple(2).TupleRad(),
            HTuple(0.1525).TupleRad(), 1.7, 2.3, 0.02,
            (HTuple("none").Append("no_pregeneration")), "use_polarity",
            ((HTuple(4).Append(11)).Append(4)), 4, &hv_ModelId);
        GetShapeModelContours(&ho_ModelContours, hv_ModelId, 1);
        SetShapeModelParam(hv_ModelId, "timeout", 10000);
      }
      catch (HalconCpp::HException &HDevExpDefaultException) {
        ROS_ERROR("Error Occured while Creating Scaled Shape Model");
        HDevWindowStack::SetActive(hv_WindowHandle_sub);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        HDevWindowStack::SetActive(hv_WindowHandle_main);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        image_subscriber_main.shutdown();
        success = false;
        as_.setAborted();
        return;
      }
      /*//////////////////////////////////////////////////
      //  Affine Transform Image ----START
      //////////////////////////////////////////////////*/
      ReadImage(&ho_Image_Match,
                HTuple(goalparam->filename_target.data.c_str()));
      HTuple hv_Matrix1, hv_Matrix2, hv_Matrix3;
      HObject ho_Image_Match_transformed;
      GetImageSize(ho_Image_Match, &hv_Width, &hv_Height);
      HomMat2dIdentity(&hv_Matrix1);
      HomMat2dRotate(hv_Matrix1, -(double)goalparam->theta_rad.data,
                     hv_Height / 2 + goalparam->alignment_offset_theta.y,
                     hv_Width / 2 + goalparam->alignment_offset_theta.x,
                     &hv_Matrix2);
      HomMat2dTranslate(
          hv_Matrix2,
          goalparam->point_offset.y / goalparam->scaling_factor.data -
              goalparam->alignment_offset.y,
          goalparam->point_offset.x / goalparam->scaling_factor.data -
              goalparam->alignment_offset.x,
          &hv_Matrix3);
      AffineTransImage(ho_Image_Match, &ho_Image_Match_transformed, hv_Matrix3,
                       "constant", "false");
      /*//////////////////////////////////////////////////
      //  Affine Transform Image ----END
      //////////////////////////////////////////////////*/

      HDevWindowStack::SetActive(hv_WindowHandle_sub);
      if (HDevWindowStack::IsOpen())
        DispObj(ho_Image_Match_transformed, HDevWindowStack::GetActive());

      GetImageSize(ho_Image_Match_transformed, &hv_Width, &hv_Height);
      CountSeconds(&hv_Seconds1);
      FindScaledShapeModel(
          ho_Image_Match_transformed, hv_ModelId, HTuple(-2).TupleRad(),
          HTuple(2).TupleRad(), 1.7, 2.3, 0.4, 1, 0.5, "least_squares",
          (HTuple(4).Append(11)), 0, &hv_ModelRow, &hv_ModelColumn,
          &hv_ModelAngle, &hv_ModelScale, &hv_ModelScore);
      CountSeconds(&hv_Seconds2);
      HDevWindowStack::SetActive(hv_WindowHandle_sub);
      halcon_bridge::dev_display_shape_matching_results(
          hv_ModelId, "red", hv_ModelRow, hv_ModelColumn, hv_ModelAngle,
          hv_ModelScale, hv_ModelScale, 0);
      halcon_bridge::disp_message(
          hv_WindowHandle_sub,
          ((HTuple(hv_ModelScore.TupleLength()) + " Match found in ") +
           (((hv_Seconds2 - hv_Seconds1) * 1000.0).TupleString(".1f"))) +
              " ms",
          "window", -1, -1, "red", "false");

      // Matching 01: transform the model contours into the detected positions
      if (0 != ((hv_ModelScore.TupleLength()) != 0)) {
        //
        hv_deltax = ((hv_ModelColumn - (hv_Width / 2)) * hv_pix_to_step_factor);
        hv_deltay = ((hv_ModelRow - (hv_Height / 2)) * hv_pix_to_step_factor);
        hv_movevector_x = ((hv_ModelAngle.TupleCos()) * hv_deltax) -
                          ((hv_ModelAngle.TupleSin()) * hv_deltay);
        hv_movevector_y = ((hv_ModelAngle.TupleSin()) * hv_deltax) +
                          ((hv_ModelAngle.TupleCos()) * hv_deltay);
        TupleCeil(hv_movevector_x, &hv_movevector_x_c);
        TupleCeil(hv_movevector_y, &hv_movevector_y_c);
        if (0 != (hv_movevector_x_c == 0.0)) {
          hv_movevector_x_c = 1;
        }
        if (0 != (hv_movevector_y_c == 0.0)) {
          hv_movevector_y_c = 1;
        }
        geometry_msgs::Point point;
        std_srvs::Empty emp_srv;

        point.x = (double)hv_movevector_x_c;
        point.y = (double)hv_movevector_y_c;
        stamper_sample_xy_cmd_stp.publish(point);
        stamper_sample_xy_wait_for_stop_client.call(emp_srv);

        if (0 != (HTuple((hv_movevector_x_c.TupleAbs()) < alignment_thresh_stp)
                      .TupleAnd((hv_movevector_y_c.TupleAbs()) <
                                alignment_thresh_stp))) {
          break;
        }
      } else {
        geometry_msgs::Point point;
        std_srvs::Empty emp_srv;
        point.x = (double)hv_uzumaki_vector_x[hv_cycle];
        point.y = (double)hv_uzumaki_vector_y[hv_cycle];
        stamper_sample_xy_cmd_stp.publish(point);
        stamper_sample_xy_wait_for_stop_client.call(emp_srv);
        hv_cycle += 1;
      }
      ClearShapeModel(hv_ModelId);
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        HDevWindowStack::SetActive(hv_WindowHandle_main);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        HDevWindowStack::SetActive(hv_WindowHandle_sub);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        image_subscriber_main.shutdown();
        return;
      }
      ros::spinOnce();
    }
    HDevWindowStack::SetActive(hv_WindowHandle_main);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

    success = true;
    as_.setSucceeded();
    image_subscriber_main.shutdown();
    return;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tdmms_autostamp_flake_aligner_xy10x_fine_action");

  autostamp_flake_aligner_xy10x_fine_class flake_aligner(
      ros::this_node::getName());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  ros::spin();

  return 0;
}
