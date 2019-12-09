// Copyright 2016 by S. Masubuchi
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <stdio.h>
#include <stdlib.h>

#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_flake_aligner_ncc_action/AlignFlakeNCCAction.h>
#include <geometry_msgs/Pose.h>
#include <adm2/velocity.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>

#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"

using namespace HalconCpp;
class autostamp_flake_aligner_ncc_class {
 protected:
  ros::NodeHandle node_;
  ros::Subscriber image_subscriber_main;
  actionlib::SimpleActionServer<
      tdmms_autostamp_flake_aligner_ncc_action::AlignFlakeNCCAction> as_;
  std::string action_name_;
  tdmms_autostamp_flake_aligner_ncc_action::AlignFlakeNCCFeedback feedback_;
  tdmms_autostamp_flake_aligner_ncc_action::AlignFlakeNCCResult result_;
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
  double cyclemove_deltay_stp_5x;
  double cyclemove_deltax_stp_5x;
  double alignment_thresh_stp;

 public:
  autostamp_flake_aligner_ncc_class(std::string name)
      : as_(node_, name,
            boost::bind(&autostamp_flake_aligner_ncc_class::executeAlignFlake,
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
    parameter_name = "/tdmms_stamper_autostamp/om_pixel_to_step_factor_5x";
    if (!node_.getParam(parameter_name, om_pixel_to_step_factor)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }
    parameter_name = "/tdmms_stamper_autostamp/cyclemove_deltax_stp_5x";
    if (!node_.getParam(parameter_name, cyclemove_deltax_stp_5x)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }
    parameter_name = "/tdmms_stamper_autostamp/cyclemove_deltay_stp_5x";
    if (!node_.getParam(parameter_name, cyclemove_deltay_stp_5x)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }
    parameter_name = "/tdmms_stamper_autostamp/alignment_thresh_stp_5x";
    if (!node_.getParam(parameter_name, alignment_thresh_stp)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }

    as_.start();
  }

  ~autostamp_flake_aligner_ncc_class(void) {}

  void imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image) {
    halcon_bridge::toHObject(Image, &ho_Image_buf);
    ho_Image = ho_Image_buf;
  }

  void executeAlignFlake(const tdmms_autostamp_flake_aligner_ncc_action::
                             AlignFlakeNCCGoalConstPtr &goalparam) {
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
        &autostamp_flake_aligner_ncc_class::imageStreamCallback, this);

    ros::Duration(1).sleep();

    ////////////////////////////////////////
    ////////////////////////////////////////
    std_msgs::UInt8 pos;
    pos.data = 3;
    stamper_nikon_revolv_set.publish(pos);

    //////////////////////////////////////
    ///////////////////////////////////////
    std_msgs::UInt8 light_inten;
    light_inten.data = 20;
    stamper_nic_100a_cmd_inten.publish(light_inten);

    ///////////////////////////////////////
    //////////////////////////////////////
    std_msgs::UInt32 fpos;
    fpos.data = goalparam->focuspos.data - 500;
    stamper_nikon_cmd_z_abs.publish(fpos);

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

    hv_pix_to_step_factor = om_pixel_to_step_factor;
    SetWindowAttr("background_color", "black");
    ReadImage(&ho_Image_Match, HTuple(goalparam->filename_target.data.c_str()));

    hv_f0 = 0;
    hv_f1 = 1;
    for (hv_i = 0; hv_i <= 100; hv_i += 1) {
      hv_fib[hv_i] = hv_f0 + hv_f1;
      hv_f0 = hv_f1;
      hv_f1 = ((const HTuple &)hv_fib)[hv_i];
    }
    hv_cycle_deltax_stp = cyclemove_deltax_stp_5x;
    hv_cycle_deltay_stp = cyclemove_deltay_stp_5x;

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
    HTuple hv_scale_factor_x = goalparam->scale_factor_x.data;
    HTuple hv_scale_factor_y = goalparam->scale_factor_y.data;
    HTuple hv_HomMat2DIdentity;
    HTuple hv_HomMat2DScale;

    HObject ho_ImageAffinTrans;
    HTuple hv_Row;
    HTuple hv_Column;
    HTuple hv_Angle;
    HTuple hv_Score;
    HTuple hv_MatchingObjIdx;
    HObject ho_TransContours;
    HObject ho_ImageIlluminated, ho_ImageGray;
    ReadImage(&ho_Image_Match, HTuple(goalparam->filename_target.data.c_str()));
    Rgb1ToGray(ho_Image_Match, &ho_Image_Match);

    /*//////////////////////////////////////////////////
    //  Affine Transform Image ----START
    //////////////////////////////////////////////////*/
    HTuple hv_Matrix1, hv_Matrix2, hv_Matrix3;
    HObject ho_Image_Match_transformed;
    GetImageSize(ho_Image_Match, &hv_Width, &hv_Height);
    HomMat2dIdentity(&hv_Matrix1);
    HomMat2dRotate(hv_Matrix1, -static_cast<double>(goalparam->theta_rad.data),
                   hv_Height / 2, hv_Width / 2, &hv_Matrix2);
    HomMat2dTranslate(hv_Matrix2, goalparam->point_offset.y,
                      goalparam->point_offset.x, &hv_Matrix3);
    AffineTransImage(ho_Image_Match, &ho_Image_Match_transformed, hv_Matrix3,
                     "constant", "true");
    /*//////////////////////////////////////////////////
    //  Affine Transform Image ----END
    //////////////////////////////////////////////////*/

    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_Image_Match_transformed, HDevWindowStack::GetActive());
    SetDraw(hv_WindowHandle_sub, "margin");
    HTuple hv_margin;
    hv_margin = 250;
    GenRectangle1(&ho_ModelRegion, hv_Height / 2 - hv_margin,
                  hv_Width / 2 - hv_margin, hv_Height / 2 + hv_margin,
                  hv_Width / 2 + hv_margin);
    DispRectangle1(hv_WindowHandle_sub, hv_Height / 2 - hv_margin,
                   hv_Width / 2 - hv_margin, hv_Height / 2 + hv_margin,
                   hv_Width / 2 + hv_margin);
    GenCrossContourXld(&ho_TransContours, hv_Height / 2, hv_Width / 2, 20, 0);
    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_TransContours, HDevWindowStack::GetActive());

    ReduceDomain(ho_Image_Match_transformed, ho_ModelRegion, &ho_TemplateImage);
    CreateNccModel(ho_TemplateImage, "auto", HTuple(0).TupleRad(),
                   HTuple(0).TupleRad(), "auto", "use_polarity", &hv_ModelId);

    while (0 != 1) {
      try {
        ros::Duration(0.1).sleep();

        Rgb1ToGray(ho_Image, &ho_ImageGray);
        ScaleImageMax(ho_ImageGray, &ho_ImageGray);
        HDevWindowStack::SetActive(hv_WindowHandle_main);
        if (HDevWindowStack::IsOpen())
          DispObj(ho_ImageGray, HDevWindowStack::GetActive());

        SetSystem("border_shape_models", "true");

        HomMat2dIdentity(&hv_HomMat2DIdentity);
        GetImageSize(ho_Image, &hv_Width, &hv_Height);

        HomMat2dScale(hv_HomMat2DIdentity, hv_scale_factor_x, hv_scale_factor_y,
                      0, 0, &hv_HomMat2DScale);
        AffineTransImageSize(ho_ImageGray, &ho_ImageAffinTrans,
                             hv_HomMat2DScale, "nearest_neighbor",
                             hv_Width * hv_scale_factor_x,
                             hv_Height * hv_scale_factor_y);
        FindNccModel(ho_ImageAffinTrans, hv_ModelId, HTuple(0).TupleRad(),
                     HTuple(0).TupleRad(), 0.3, 1, 0, "true", 0, &hv_Row,
                     &hv_Column, &hv_Angle, &hv_Score);
        HTuple end_val16 = (hv_Score.TupleLength()) - 1;
        HTuple step_val16 = 1;
        for (hv_MatchingObjIdx = 0;
             hv_MatchingObjIdx.Continue(end_val16, step_val16);
             hv_MatchingObjIdx += step_val16) {
          // Matching 01: display the center of the match
          GenCrossContourXld(&ho_TransContours, hv_Row / 2, hv_Column / 2, 20,
                             hv_Angle);
          HDevWindowStack::SetActive(hv_WindowHandle_main);
          if (HDevWindowStack::IsOpen())
            DispObj(ho_TransContours, HDevWindowStack::GetActive());
        }

        if (0 != ((hv_Score.TupleLength()) != 0)) {
          hv_deltax =
              -((hv_Column / 2 - (hv_Width / 2)) * hv_pix_to_step_factor / 10);
          hv_deltay =
              -((hv_Row / 2 - (hv_Height / 2)) * hv_pix_to_step_factor / 10);
          hv_movevector_x = ((hv_Angle.TupleCos()) * hv_deltax) -
                            ((hv_Angle.TupleSin()) * hv_deltay);
          hv_movevector_y = ((hv_Angle.TupleSin()) * hv_deltax) +
                            ((hv_Angle.TupleCos()) * hv_deltay);
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
          ros::Duration(0.5).sleep();
          if (0 != (HTuple((hv_movevector_x_c.TupleAbs()) < 2)
                        .TupleAnd((hv_movevector_y_c.TupleAbs()) < 2))) {
            break;
          }
        } else {
          geometry_msgs::Point point;
          std_srvs::Empty emp_srv;
          point.x = (double)hv_uzumaki_vector_x[hv_cycle];
          point.y = (double)hv_uzumaki_vector_y[hv_cycle];
        }
        if (as_.isPreemptRequested() || !ros::ok()) {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          HDevWindowStack::SetActive(hv_WindowHandle_main);
          if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
          HDevWindowStack::SetActive(hv_WindowHandle_sub);
          if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
          image_subscriber_main.shutdown();
          ClearNccModel(hv_ModelId);
          return;
        }
      }
      catch (HalconCpp::HException &HDevExpDefaultException) {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        // set the action state to preempted
        as_.setAborted();
        HDevWindowStack::SetActive(hv_WindowHandle_main);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        HDevWindowStack::SetActive(hv_WindowHandle_sub);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        image_subscriber_main.shutdown();
        ClearNccModel(hv_ModelId);
        return;
      }
      ros::spinOnce();
    }
    HDevWindowStack::SetActive(hv_WindowHandle_main);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

    image_subscriber_main.shutdown();
    ClearNccModel(hv_ModelId);
    as_.setSucceeded();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tdmms_autostamp_flake_aligner_ncc_action");

  autostamp_flake_aligner_ncc_class flake_aligner(ros::this_node::getName());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  ros::spin();

  return 0;
}
