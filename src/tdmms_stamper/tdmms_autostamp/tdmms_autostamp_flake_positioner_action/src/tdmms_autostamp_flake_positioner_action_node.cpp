// Copyright 2016 by S. Masubuchi

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>

#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_flake_positioner_action/PositionFlakeAction.h>
#include <stamper_sample_xy/velocity.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"

using namespace HalconCpp;
class autostamp_flake_positioner_class {
 protected:
  ros::NodeHandle node_;
  ros::Publisher transfer_valve_open_publisher;
  ros::Publisher transfer_valve_close_publisher;
  ros::Publisher stamper_sample_xy_cmd_vel;
  ros::Publisher stamper_sample_xy_cmd_abs;
  ros::Publisher stamper_sample_xy_cmd_stop;
  ros::Publisher stamper_sample_xy_cmd_stp;
  ros::Publisher stamper_nic_100a_cmd_inten;
  ros::Publisher stamper_nikon_cmd_z_abs;
  ros::Publisher stamper_nikon_revolv_set;
  ros::Subscriber image_subscriber_main;
  ros::ServiceClient stamper_sample_xy_wait_for_stop_client;

  actionlib::SimpleActionServer<
      tdmms_autostamp_flake_positioner_action::PositionFlakeAction> as_;
  std::string action_name_;
  tdmms_autostamp_flake_positioner_action::PositionFlakeFeedback feedback_;
  tdmms_autostamp_flake_positioner_action::PositionFlakeResult result_;
  HTuple hv_AcqHandle, hv_DeviceHandle, hv_Index, hv_DeviceName;
  HTuple hv_DeviceVendor;
  HObject ho_Image, ho_Image_buf;
  HTuple hv_WindowHandle_main, hv_WindowHandle_sub, hv_DeviceIdentifiers;

  int edge_detection_start_pos_x, edge_detection_start_pos_y;
  double edge_detection_pixel_to_stp_factor;

 public:
  autostamp_flake_positioner_class(std::string name)
      : as_(node_, name,
            boost::bind(&autostamp_flake_positioner_class::executePositionFlake,
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
    stamper_sample_xy_cmd_vel = node_.advertise<stamper_sample_xy::velocity>(
        "/stamper_sample_xy_master/cmd_vel", 1);
    stamper_sample_xy_cmd_abs = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_abs_move", 1);
    stamper_sample_xy_cmd_stop = node_.advertise<std_msgs::Empty>(
        "/stamper_sample_xy_master/cmd_stop", 1);
    stamper_sample_xy_cmd_stp = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_stp_move", 1);
    stamper_sample_xy_wait_for_stop_client =
        node_.serviceClient<std_srvs::Empty>(
            "/stamper_sample_xy_master/wait_for_stop");
    stamper_nikon_revolv_set = node_.advertise<std_msgs::UInt8>(
        "/stamper_nikon_master/cmd_revolv_set", 1);
    stamper_nic_100a_cmd_inten =
        node_.advertise<std_msgs::UInt8>("/stamper_nic_100a/cmd_inten", 1);
    stamper_nikon_cmd_z_abs =
        node_.advertise<std_msgs::UInt32>("/stamper_nikon_master/cmd_z_abs", 1);

    std::string parameter_name;
    parameter_name = "/tdmms_stamper_autostamp/edge_detection_start_pos_x";
    if (!node_.getParam(parameter_name, edge_detection_start_pos_x)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }

    parameter_name = "/tdmms_stamper_autostamp/edge_detection_start_pos_y";
    if (!node_.getParam(parameter_name, edge_detection_start_pos_y)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }
    parameter_name =
        "/tdmms_stamper_autostamp/edge_detection_pixel_to_stp_factor";
    if (!node_.getParam(parameter_name, edge_detection_pixel_to_stp_factor)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }

    edge_detection_pixel_to_stp_factor;
    as_.start();
  }

  ~autostamp_flake_positioner_class(void) {}

  void imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image) {
    halcon_bridge::toHObject(Image, &ho_Image_buf);
    ho_Image = ho_Image_buf;
  }

  void executePositionFlake(
      const tdmms_autostamp_flake_positioner_action::PositionFlakeGoalConstPtr &
          goalparam) {
    bool success = true;
    //as_.acceptNewGoal();
    std::string initfname_small;
    initfname_small = getenv("HOME");
    initfname_small = initfname_small + "/img744x480.jpg";
    ReadImage(&ho_Image, initfname_small.c_str());
    HalconCpp::SetWindowAttr("background_color", "black");
    HalconCpp::SetWindowAttr("window_title", "Edge Detection");

    HalconCpp::OpenWindow(0, 744*1.5, 744 * 1.5, 480 * 1.5, 0, "", "",
                          &hv_WindowHandle_main);
    HalconCpp::HDevWindowStack::Push(hv_WindowHandle_main);
    HalconCpp::SetPart(hv_WindowHandle_main, 0, 0, 480, 744);
    halcon_bridge::set_display_font(hv_WindowHandle_main, 16, "mono", "true",
                                    "false");

    std::string initfname_large;
    initfname_large = getenv("HOME");
    initfname_large = initfname_small + "/img2040x1086.jpg";
    ReadImage(&ho_Image, initfname_large.c_str());
    HalconCpp::SetWindowAttr("window_title", "Origin");
    HalconCpp::OpenWindow(0, 2560, 744 * 1.5, 480 * 1.5, 0, "", "",
                          &hv_WindowHandle_sub);
    HalconCpp::HDevWindowStack::Push(hv_WindowHandle_sub);
    HalconCpp::SetPart(hv_WindowHandle_sub, 0, 0, 1086, 2040);
    halcon_bridge::set_display_font(hv_WindowHandle_sub, 16, "mono", "true",
                                    "false");

    image_subscriber_main = node_.subscribe(
        "/autostamp_camera_streamer/image_main", 100,
        &autostamp_flake_positioner_class::imageStreamCallback, this);

    ros::Duration(1).sleep();

    ////////////////////////////////////////
    /// 光学顕微鏡レボルバーの位置を５倍に設定
    ////////////////////////////////////////
    std_msgs::UInt8 pos;
    pos.data = 1;
    stamper_nikon_revolv_set.publish(pos);
    ros::Duration(0.1).sleep();
    //////////////////////////////////////
    ///// 光源の強さを５に設定
    ///////////////////////////////////////
    std_msgs::UInt8 light_inten;
    light_inten.data = 5;
    stamper_nic_100a_cmd_inten.publish(light_inten);
    ros::Duration(0.1).sleep();
    ///////////////////////////////////////
    ////// 光学顕微鏡Zステージの位置を設定
    //////////////////////////////////////
    stamper_nikon_cmd_z_abs.publish(goalparam->focuspos);

    geometry_msgs::Point point;
    std_srvs::Empty emp_srv;

    // Local iconic variables
    HObject ho_ImageModel, ho_ImageR, ho_ImageG;
    HObject ho_ImageB, ho_Region, ho_RegionDr, ho_RegionDf;
    HObject ho_skeleton, ho_contours, ho_ImageModelGray, ho_ModelRegion;
    HObject ho_ImageModelGrayReduced, ho_ModelContours, ho_ImageGrayModel;
    HObject ho_TemplateImage, ho_RegionR, ho_Rectangle2;
    HObject ho_selected, ho_ImageGray, ho_ImageGrey, ho_TransContours;

    // Local control variables
    HTuple hv_tuple_filename, hv_i, hv_num, hv_ind;
    HTuple hv_Area, hv_Row, hv_Column, hv_ModelId;
    HTuple hv_Length, hv_Length_sum, hv_Length_tuple, hv_Exception;
    HTuple hv_LengthMax, hv_Protocol, hv_Timeout, hv_Socket;
    HTuple hv_Address, hv_To, hv_pix_to_step_factor, hv_deltay_prev;
    HTuple hv_deltax_prev;
    HTuple hv_origin_pos_x, hv_origin_pos_y;
    HTuple hv_target_pos_x, hv_target_pos_y, hv_Width, hv_Height;
    HTuple hv_area, hv_row, hv_col, hv_deltax_c, hv_deltay_c;
    HTuple hv_length, hv_length_max, hv_index, hv_isInside;
    HTuple hv_origin_row, hv_origin_column, hv_naiseki, hv_row_end;
    HTuple hv_col_end, hv_deltax, hv_deltay, hv_Seconds1, hv_ModelRow;
    HTuple hv_ModelColumn, hv_ModelAngle, hv_ModelScale, hv_ModelScore;
    HTuple hv_Seconds2, hv_movevector_x, hv_movevector_y, hv_rotangle;
    HTuple hv_r_movevector_x, hv_r_movevector_y, hv_MatchingObjIdx;
    HTuple hv_r_movevector_x_c, hv_r_movevector_y_c;

    int detected_count = 0;
    int i;

    SetSystem("border_shape_models", "true");

    //  Intialize Halcon Parameters
    hv_pix_to_step_factor = edge_detection_pixel_to_stp_factor;
    hv_deltay_prev = 0;
    hv_deltax_prev = 0;
    ReadImage(&ho_ImageModel, HTuple(goalparam->filename_origin.data.c_str()));
    Rgb1ToGray(ho_ImageModel, &ho_ImageGrayModel);
    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_ImageModel, HDevWindowStack::GetActive());

    //原点移動
    point.x = edge_detection_start_pos_x;
    point.y = edge_detection_start_pos_y;
    stamper_sample_xy_cmd_abs.publish(point);
    stamper_sample_xy_wait_for_stop_client.call(emp_srv);

    // SetSystem("border_shape_models", "true");
    GenRectangle1(&ho_ModelRegion, 200, 400, 900, 1600);
    ReduceDomain(ho_ImageGrayModel, ho_ModelRegion, &ho_TemplateImage);
    CreateScaledShapeModel(
        ho_TemplateImage, 8, HTuple(-180).TupleRad(), HTuple(360).TupleRad(),
        HTuple(0.1452).TupleRad(), 0.81, 0.93, 0.0025,
        (HTuple("point_reduction_medium").Append("no_pregeneration")),
        "use_polarity", ((HTuple(18).Append(32)).Append(86)), 4, &hv_ModelId);
    GetShapeModelContours(&ho_ModelContours, hv_ModelId, 1);

    AreaCenter(ho_ModelRegion, &hv_Area, &hv_Row, &hv_Column);
    halcon_bridge::dev_display_shape_matching_results(
        hv_ModelId, "red", hv_Row, hv_Column, 0.0, 1.0, 1.0, 0);

    HDevWindowStack::SetActive(hv_WindowHandle_main);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_ImageModel, HDevWindowStack::GetActive());

    hv_origin_pos_x = static_cast<int>(goalparam->point_origin.x);
    hv_origin_pos_y = static_cast<int>(goalparam->point_origin.y);

    hv_target_pos_x = static_cast<int>(goalparam->point_target.x);
    hv_target_pos_y = static_cast<int>(goalparam->point_target.y);
    ROS_INFO("startpoint:%d, %d, target: %d, %d", (int)hv_origin_pos_x,
             (int)hv_origin_pos_y, (int)hv_target_pos_x, (int)hv_target_pos_y);

    int count_of_false_detection = 0;
    while (0 != 1) {
      try {
        ros::spinOnce();

        GetImageSize(ho_Image, &hv_Width, &hv_Height);
        Decompose3(ho_Image, &ho_ImageR, &ho_ImageG, &ho_ImageB);
        Threshold(ho_ImageG, &ho_Region, 95, 255);
        Threshold(ho_ImageR, &ho_RegionR, 70, 255);
        Intersection(ho_Region, ho_RegionR, &ho_Region);

        ClosingCircle(ho_Region, &ho_Region, 130);
        DilationCircle(ho_Region, &ho_RegionDr, 3);
        Difference(ho_RegionDr, ho_Region, &ho_RegionDf);
        GenRectangle1(&ho_Rectangle2, 30, 30, hv_Height - 30, hv_Width - 30);
        Intersection(ho_RegionDf, ho_Rectangle2, &ho_RegionDf);
        Skeleton(ho_RegionDf, &ho_skeleton);
        GenContoursSkeletonXld(ho_skeleton, &ho_contours, 1, "filter");
        CountObj(ho_contours, &hv_num);
        AreaCenter(ho_Region, &hv_area, &hv_row, &hv_col);

        if (0 != (hv_num == 0)) {
          HDevWindowStack::SetActive(hv_WindowHandle_main);
          if (HDevWindowStack::IsOpen())
            DispObj(ho_Image, HDevWindowStack::GetActive());
          point.x = static_cast<double>(goalparam->delta_stp.x);
          point.y = static_cast<double>(goalparam->delta_stp.y);
          stamper_sample_xy_cmd_stp.publish(point);
          stamper_sample_xy_wait_for_stop_client.call(emp_srv);
        } else {
          LengthXld(ho_contours, &hv_length);
          TupleMax(hv_length, &hv_length_max);
          TupleFind(hv_length, hv_length_max, &hv_index);
          SelectObj(ho_contours, &ho_selected, hv_index + 1);

          GetContourXld(ho_selected, &hv_row, &hv_col);
          TupleLength(hv_row, &hv_Length);

          TestRegionPoint(ho_RegionDr, 0, 0, &hv_isInside);
          if (0 != (hv_isInside == 1)) {
            hv_origin_row = 0;
            hv_origin_column = 0;
          }
          TestRegionPoint(ho_RegionDr, hv_Height - 1, hv_Width - 1,
                          &hv_isInside);
          if (0 != (hv_isInside == 1)) {
            hv_origin_row = hv_Height - 1;
            hv_origin_column = hv_Width - 1;
          }
          TestRegionPoint(ho_RegionDr, 0, hv_Width - 1, &hv_isInside);
          if (0 != (hv_isInside == 1)) {
            hv_origin_row = 0;
            hv_origin_column = hv_Width - 1;
          }
          TestRegionPoint(ho_RegionDr, hv_Height - 1, 0, &hv_isInside);
          if (0 != (hv_isInside == 1)) {
            hv_origin_row = hv_Height - 1;
            hv_origin_column = 0;
          }
          hv_naiseki = ((HTuple(hv_row[0]) - hv_origin_row) *
                        (HTuple(hv_col[hv_Length - 1]) - hv_origin_column)) -
                       ((HTuple(hv_col[0]) - hv_origin_column) *
                        (HTuple(hv_row[hv_Length - 1]) - hv_origin_row));

          if (0 != (hv_naiseki < 0)) {
            hv_row_end = ((const HTuple &)hv_row)[hv_Length - 1];
            hv_col_end = ((const HTuple &)hv_col)[hv_Length - 1];
          } else {
            hv_row_end = ((const HTuple &)hv_row)[0];
            hv_col_end = ((const HTuple &)hv_col)[0];
          }

          hv_deltax =
              (-(hv_col_end - (hv_Width / 2)) * hv_pix_to_step_factor / 2);
          hv_deltay =
              ((-(hv_row_end - (hv_Height / 2))) * hv_pix_to_step_factor / 2);

          TupleCeil(hv_deltax, &hv_deltax_c);
          TupleCeil(hv_deltay, &hv_deltay_c);
          if (0 != (hv_deltax_c == -0.0)) {
            hv_deltax_c = 0;
          }
          if (0 != (hv_deltay_c == -0.0)) {
            hv_deltay_c = 0;
          }
          //Emphasize(ho_Image, &ho_Image, 10, 10, 4);  ///   Emphasize Image
                                                      /// added 2016.5.27 by
                                                      /// S. Masubuchi
          //ShockFilter(ho_Image, &ho_Image, 0.5, 10, "canny", 1);
          Rgb1ToGray(ho_Image, &ho_ImageGray);
          HObject ho_ImageGray_Mult;
          MultImage(ho_ImageGray, ho_ImageGray, &ho_ImageGray_Mult, 0.010, 0);
          HDevWindowStack::SetActive(hv_WindowHandle_main);
          if (HDevWindowStack::IsOpen())
            DispObj(ho_ImageGray_Mult, HDevWindowStack::GetActive());

          CountSeconds(&hv_Seconds1);
          FindScaledShapeModel(
              ho_ImageGray_Mult, hv_ModelId, HTuple(-180).TupleRad(),
              HTuple(360).TupleRad(), 0.81, 0.93, 0.5, 1, 0.5, "least_squares",
              (HTuple(8).Append(0)), 0, &hv_ModelRow, &hv_ModelColumn,
              &hv_ModelAngle, &hv_ModelScale, &hv_ModelScore);
          CountSeconds(&hv_Seconds2);
          HDevWindowStack::SetActive(hv_WindowHandle_main);
          halcon_bridge::dev_display_shape_matching_results(
              hv_ModelId, "red", hv_ModelRow, hv_ModelColumn, hv_ModelAngle,
              hv_ModelScale, hv_ModelScale, 0);
          halcon_bridge::disp_message(
              hv_WindowHandle_main,
              ((HTuple(hv_ModelScore.TupleLength()) + " Match found in ") +
               (((hv_Seconds2 - hv_Seconds1) * 1000.0).TupleString(".1f"))) +
                  " ms",
              "window", -1, -1, "red", "false");

          if (0 != (HTuple((hv_ModelRow.TupleLength()) != 0)
                        .TupleAnd(hv_ModelScore > 0.5))) {
            hv_deltax =
                ((-(hv_ModelColumn - (hv_Width / 2))) * hv_pix_to_step_factor) /
                20;
            hv_deltay =
                ((-(hv_ModelRow - (hv_Height / 2))) * hv_pix_to_step_factor) /
                20;

            TupleCeil(hv_deltax, &hv_deltax_c);
            TupleCeil(hv_deltay, &hv_deltay_c);
            if (0 != (hv_deltax_c == 0.0)) {
              hv_deltax_c = 1;
            }
            if (0 != (hv_deltay_c == 0.0)) {
              hv_deltay_c = 1;
            }

            point.x = static_cast<double>(hv_deltax_c);
            point.y = static_cast<double>(hv_deltay_c);
            stamper_sample_xy_cmd_stp.publish(point);
            stamper_sample_xy_wait_for_stop_client.call(emp_srv);

            detected_count++;
            if (0 != (HTuple((hv_deltax_c.TupleAbs()) < 3)
                          .TupleAnd((hv_deltay_c.TupleAbs()) < 5))) {
              break;
            }
            if (detected_count >1)
              break;
          } else {
            point.x = static_cast<double>(hv_deltax_c);
            point.y = static_cast<double>(hv_deltay_c);
            stamper_sample_xy_cmd_stp.publish(point);
            stamper_sample_xy_wait_for_stop_client.call(emp_srv);
            detected_count = 0;
            count_of_false_detection++;
            ROS_INFO("FalseDetection: %d", count_of_false_detection);
          }
          //////////////////////////////////////////////
          // Error Handling ここで設定した回数以上検出
          // に失敗した場合次のエッジに行く
          /////////////////////////////////////////////
          if ( count_of_false_detection > 150 ) {
            ROS_INFO("Failed to detect edges");
            as_.setAborted();
            success = false;
            HDevWindowStack::SetActive(hv_WindowHandle_sub);
            if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
            HDevWindowStack::SetActive(hv_WindowHandle_main);
            if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
            return;
          }
        }
      }
      catch (HalconCpp::HException &HDevExpDefaultException) {
        continue;
      }

      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        HDevWindowStack::SetActive(hv_WindowHandle_sub);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        HDevWindowStack::SetActive(hv_WindowHandle_main);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        return;
      }
    }

    hv_movevector_x = hv_target_pos_x - hv_origin_pos_x;
    hv_movevector_y = hv_target_pos_y - hv_origin_pos_y;
    hv_rotangle = hv_ModelAngle + HTuple(3).TupleRad();
    // カメラの回転角補正・適宜微調整が必要
    ROS_INFO("movevector:%f, %f, rotangle:%f", (double)hv_movevector_x,
             (double)hv_movevector_y, (double)hv_rotangle);
    hv_r_movevector_x = ((hv_rotangle.TupleCos()) * hv_movevector_x) -
                        ((hv_rotangle.TupleSin()) * hv_movevector_y);
    hv_r_movevector_y = ((hv_rotangle.TupleSin()) * hv_movevector_x) +
                        ((hv_rotangle.TupleCos()) * hv_movevector_y);
    ROS_INFO("movevector_rotated:%f, %f, rotangle:%f",
             (double)hv_r_movevector_x, (double)hv_r_movevector_y,
             (double)hv_rotangle);
    TupleCeil(hv_r_movevector_x, &hv_r_movevector_x_c);
    TupleCeil(hv_r_movevector_y, &hv_r_movevector_y_c);
    if (0 != (hv_r_movevector_x_c == -0.0)) {
      hv_r_movevector_x_c = 0.0;
    }
    if (0 != (hv_r_movevector_y_c == 0.0)) {
      hv_r_movevector_y_c = 0.0;
    }

    point.x = (-1) * static_cast<double>(hv_r_movevector_x_c) * 100;
    point.y = static_cast<double>(hv_r_movevector_y_c) * 100;

    stamper_sample_xy_cmd_stp.publish(point);
    stamper_sample_xy_wait_for_stop_client.call(emp_srv);

    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    HDevWindowStack::SetActive(hv_WindowHandle_main);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());

    image_subscriber_main.shutdown();

    // result_.PositionFlakeResult = feedback_.PositionFlakeFeedback;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tdmms_autostamp_flake_positioner_action");
  autostamp_flake_positioner_class flake_positioner(ros::this_node::getName());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  ros::spin();
  return 0;
}
