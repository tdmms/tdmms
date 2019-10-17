// Copyright 2016 by S. Masubuchi
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>

#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_flake_aligner_xytheta5x2_action/AlignFlakeXYTheta5x2Action.h>
#include <geometry_msgs/Pose.h>
#include <adm2/velocity.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>
#include <QtNetwork>

#include <string>
#include <algorithm>
#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"

#include <stdio.h>
#include <stdlib.h>

using namespace HalconCpp;
class autostamp_flake_aligner_xytheta5x2_class {
 protected:
  ros::NodeHandle node_;
  ros::Subscriber image_subscriber_main;
  actionlib::SimpleActionServer<
      tdmms_autostamp_flake_aligner_xytheta5x2_action::AlignFlakeXYTheta5x2Action>
      as_;
  std::string action_name_;
  tdmms_autostamp_flake_aligner_xytheta5x2_action::AlignFlakeXYTheta5x2Feedback
      feedback_;
  tdmms_autostamp_flake_aligner_xytheta5x2_action::AlignFlakeXYTheta5x2Result
      result_;
  HTuple hv_AcqHandle, hv_DeviceHandle, hv_Index, hv_DeviceName;
  HTuple hv_DeviceVendor;
  HObject ho_Image, ho_Image_buf;
  HTuple hv_WindowHandle_main, hv_WindowHandle_sub, hv_DeviceIdentifiers;
  ros::Publisher stamper_sample_xy_cmd_vel;
  ros::Publisher stamper_sample_xy_cmd_abs;
  ros::Publisher stamper_sample_xy_cmd_stp;
  ros::ServiceClient stamper_sample_xy_wait_for_stop_client;
  ros::Publisher stamper_sample_theta_cmd_vel;
  ros::Publisher stamper_sample_theta_cmd_abs;
  ros::Publisher stamper_sample_theta_cmd_stp;
  ros::Publisher stamper_nikon_revolv_set;
  ros::Publisher stamper_nikon_cmd_z_abs;
  ros::Publisher stamper_nic_100a_cmd_inten;
  ros::ServiceClient stamper_sample_theta_wait_for_stop_client;

  double om_pixel_to_step_factor;
  double cyclemove_deltay_stp_5x;
  double cyclemove_deltax_stp_5x;
  double alignment_thresh_stp;

  double pi_const;

 public:
  autostamp_flake_aligner_xytheta5x2_class(std::string name)
      : as_(node_, name,
            boost::bind(
                &autostamp_flake_aligner_xytheta5x2_class::executeAlignFlake,
                this, _1),
            false),
        action_name_(name) {
    pi_const = 3.14159265358979323846;

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

    stamper_sample_xy_cmd_vel =
        node_.advertise<adm2::velocity>("/stamper_sample_xy_master/cmd_vel", 1);
    stamper_sample_xy_cmd_abs = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_abs_move", 1);
    stamper_sample_xy_cmd_stp = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_stp_move", 1);
    stamper_sample_xy_wait_for_stop_client =
        node_.serviceClient<std_srvs::Empty>(
            "/stamper_sample_xy_master/wait_for_stop");

    stamper_sample_theta_cmd_vel = node_.advertise<adm2::velocity>(
        "/stamper_sample_theta_master/cmd_vel", 1);
    stamper_sample_theta_cmd_abs = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_theta_master/cmd_abs_move", 1);
    stamper_sample_theta_cmd_stp = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_theta_master/cmd_stp_move", 1);
    stamper_sample_theta_wait_for_stop_client =
        node_.serviceClient<std_srvs::Empty>(
            "/stamper_sample_theta_master/wait_for_stop");

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

  ~autostamp_flake_aligner_xytheta5x2_class(void) {}

  void imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image) {
    halcon_bridge::toHObject(Image, &ho_Image_buf);
    ho_Image = ho_Image_buf;
  }

  double roundAngle(double angle_rad) {
    double angle_new;
    angle_new = fmod(angle_rad, 2 * pi_const);
    angle_new = fmod(angle_new + 2 * pi_const, 2 * pi_const);
    if (angle_new > pi_const) angle_new -= 2 * pi_const;
    return angle_new;
  }

  void openWindows() {
    /////////////////////////////////
    /// Opening Windows
    //////////////////////////////////
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
  }

  void closeWindows() {
    HDevWindowStack::SetActive(hv_WindowHandle_main);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
  }

  void executeAlignFlake(const tdmms_autostamp_flake_aligner_xytheta5x2_action::
                             AlignFlakeXYTheta5x2GoalConstPtr &goalparam) {
    bool success = true;
    openWindows();
    ///////////////////////////////////
    /// Start Subscribing to Image Stream
    ///////////////////////////////////////
    image_subscriber_main = node_.subscribe(
        "/autostamp_camera_streamer/image_main", 100,
        &autostamp_flake_aligner_xytheta5x2_class::imageStreamCallback, this);

    ros::Duration(1).sleep();

    ////////////////////////////////////////
    /// Set Objective Lens Tartlet to 5X
    ////////////////////////////////////////
    std_msgs::UInt8 pos;
    pos.data = 1;
    stamper_nikon_revolv_set.publish(pos);
    ros::Duration(1).sleep();

    //////////////////////////////////////
    /// Set Light Intensity
    ///////////////////////////////////////
    std_msgs::UInt8 light_inten;
    light_inten.data = 5;
    stamper_nic_100a_cmd_inten.publish(light_inten);

    ///////////////////////////////////////
    /// Set Z stage position of Optical Microscope
    //////////////////////////////////////
    stamper_nikon_cmd_z_abs.publish(goalparam->focuspos);

    ////////////////////////
    // Local iconic variables
    /////////////////////////
    HObject ho_ModelRegion, ho_TemplateImage;
    HObject ho_ModelContours, ho_Image_Match;

    //////////////////////////
    // Local control variables
    ///////////////////////////
    HTuple hv_pix_to_step_factor;
    HTuple hv_f0, hv_f1, hv_i, hv_fib, hv_cycle_deltax_stp;
    HTuple hv_cycle_deltay_stp, hv_count, hv_k, hv_j, hv_uzumaki_vector_x;
    HTuple hv_uzumaki_vector_y, hv_cycle_deltax_stp_buf,
        hv_cycle_deltay_stp_buf;
    HTuple hv_cycle, hv_ModelId, hv_Width, hv_Height, hv_Seconds1;
    HTuple hv_ModelRow, hv_ModelColumn, hv_ModelAngle, hv_ModelScale;
    HTuple hv_ModelScore, hv_Seconds2;
    HTuple hv_deltax, hv_deltay, hv_movevector_x, hv_movevector_y;

    geometry_msgs::Point point;
    std_srvs::Empty emp_srv;

    hv_pix_to_step_factor = om_pixel_to_step_factor;
    //////////////////////////////////////////////////////////////////
    //// Generate Displacement Vectors used in Case Match Not Found
    ////////////////////////////////////////////////////////
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

    ////////////////////////////
    ///// Read Image
    ///////////////////////////
    try {
      QUrl imageURL(QString(goalparam->filename_target.data.c_str()));
      QNetworkRequest request(imageURL);
      QNetworkAccessManager m_WebCtrl;
      QEventLoop eventLoop;

      QObject::connect(&m_WebCtrl, SIGNAL(finished(QNetworkReply*)), &eventLoop, SLOT(quit()));
      QNetworkReply *reply = m_WebCtrl.get(request);
      eventLoop.exec();

      QByteArray downloadedData;
      downloadedData = reply->readAll();

      QString tempfname = "/ramdisk/tmp_stamper_align.jpg";
      QFile file(tempfname);
      file.open(QIODevice::WriteOnly);
      file.write(downloadedData);
      file.close();
      
      ReadImage(&ho_Image_Match,
                HTuple(tempfname.toStdString().c_str()));
    }
    catch (HalconCpp::HException &HDevExpDefaultException) {
      ROS_ERROR("%s: Image File Read Error !!", action_name_.c_str());
      // set the action state to preempted
      closeWindows();
      image_subscriber_main.shutdown();
      success = false;
      as_.setAborted();
      return;
    }

    /*//////////////////////////////////////////////////
    //  Affine Transform Image ----START
    //////////////////////////////////////////////////*/
    HTuple hv_Matrix1, hv_Matrix2, hv_Matrix3;
    HObject ho_Image_Match_transformed;
    GetImageSize(ho_Image_Match, &hv_Width, &hv_Height);
    HomMat2dIdentity(&hv_Matrix1);
    HomMat2dRotate(hv_Matrix1, -static_cast<double>(goalparam->theta_rad.data),
                   hv_Height / 2, hv_Width / 2, &hv_Matrix2);
    HomMat2dTranslate(hv_Matrix2, goalparam->point_offset.x / 10,
                      goalparam->point_offset.y / 10, &hv_Matrix3);
    AffineTransImage(ho_Image_Match, &ho_Image_Match_transformed, hv_Matrix3,
                     "constant", "true");
    /*//////////////////////////////////////////////////
    //  Affine Transform Image ----END
    //////////////////////////////////////////////////*/

    int dirmove;
    double deltheta_prev = 0;
    dirmove = 1;

    while (true) {
      hv_cycle = 0;
      point.x = static_cast<int>(std::min(
          std::max(static_cast<double>(100 * dirmove * deltheta_prev * 100),
                   -1000.0),
          1000.0));
      point.y = 0;
      stamper_sample_theta_cmd_stp.publish(point);
      stamper_sample_theta_wait_for_stop_client.call(emp_srv);

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
              ho_TemplateImage, 4, HTuple(0).TupleRad(), HTuple(360).TupleRad(),
              HTuple(0.1525).TupleRad(), 1.05, 1.1, 0.0027,
              (HTuple("none").Append("no_pregeneration")), "use_polarity",
              ((HTuple(11).Append(16)).Append(4)), 4, &hv_ModelId);
          GetShapeModelContours(&ho_ModelContours, hv_ModelId, 1);
          SetShapeModelParam(hv_ModelId, "timeout", 60000);
        }
        catch (HalconCpp::HException &HDevExpDefaultException) {
          ROS_INFO("%s: Aborted", action_name_.c_str());
          closeWindows();
          image_subscriber_main.shutdown();
          success = false;
          as_.setAborted();
          return;
        }

        HDevWindowStack::SetActive(hv_WindowHandle_sub);
        if (HDevWindowStack::IsOpen())
          DispObj(ho_Image_Match_transformed, HDevWindowStack::GetActive());

        GetImageSize(ho_Image_Match_transformed, &hv_Width, &hv_Height);
        CountSeconds(&hv_Seconds1);
        FindScaledShapeModel(
            ho_Image_Match_transformed, hv_ModelId, HTuple(0).TupleRad(),
            HTuple(360).TupleRad(), 1.05, 1.1, 0.4, 1, 0.5, "least_squares",
            (HTuple(4).Append(1)), 0, &hv_ModelRow, &hv_ModelColumn,
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
        ClearShapeModel(hv_ModelId);
        ///////////////////////////////////////
        ///// Align XY Offset
        ///////////////////////////////////////
        if (0 != hv_ModelScore.TupleLength()) {
          hv_deltax =
              ((hv_ModelColumn - (hv_Width / 2)) * hv_pix_to_step_factor);
          hv_deltay = ((hv_ModelRow - (hv_Height / 2)) * hv_pix_to_step_factor);
          hv_movevector_x = ((hv_ModelAngle.TupleCos()) * hv_deltax) -
                            ((hv_ModelAngle.TupleSin()) * hv_deltay);
          hv_movevector_y = ((hv_ModelAngle.TupleSin()) * hv_deltax) +
                            ((hv_ModelAngle.TupleCos()) * hv_deltay);
          geometry_msgs::Point point;
          std_srvs::Empty emp_srv;
          point.x = static_cast<int>(static_cast<double>(hv_movevector_x));
          point.y = static_cast<int>(static_cast<double>(hv_movevector_y));

          stamper_sample_xy_cmd_stp.publish(point);
          stamper_sample_xy_wait_for_stop_client.call(emp_srv);
          if ((hv_movevector_x.TupleAbs() < alignment_thresh_stp) &&
              (hv_movevector_y.TupleAbs() < alignment_thresh_stp)) {
            break;
          }
        } else {
          ///////////////////////////////////////
          ///// Model Not Found
          ///////////////////////////////////////
          geometry_msgs::Point point;
          std_srvs::Empty emp_srv;
          point.x = static_cast<int>(
              static_cast<double>(hv_uzumaki_vector_x[hv_cycle]));
          point.y = static_cast<int>(
              static_cast<double>(hv_uzumaki_vector_y[hv_cycle]));
          stamper_sample_xy_cmd_stp.publish(point);
          stamper_sample_xy_wait_for_stop_client.call(emp_srv);
          hv_cycle += 1;
        }
        if (as_.isPreemptRequested() || !ros::ok()) {
          ///////////////////////////////////////
          ///// Preempt Requested
          ///////////////////////////////////////
          ROS_INFO("%s: Preempt Requested", action_name_.c_str());
          closeWindows();
          image_subscriber_main.shutdown();
          success = false;
          as_.setPreempted();
          return;
        }
        ros::spinOnce();
      }
      ///////////////////////////////////////
      ///// Align Theta Offset
      ///////////////////////////////////////
      double deltheta;
      deltheta = static_cast<double>(hv_ModelAngle);
      if (deltheta > 3.1412) {
        deltheta = deltheta - 3.1412 * 2;
      }
      ROS_INFO("angle: %f", (double)deltheta);
      if (-0.0001 < deltheta && deltheta < 0.0001) {
        break;
      }
      if (fabs(deltheta) > fabs(deltheta_prev)) {
        dirmove = -dirmove;
      }
      deltheta_prev = deltheta;
      ros::spinOnce();
    }
    closeWindows();
    image_subscriber_main.shutdown();
    success = true;
    as_.setSucceeded();
    return;
  }
};  //  The END of Class Definition

int main(int argc, char **argv) {
  QCoreApplication a(argc, argv);
  ros::init(argc, argv, "tdmms_autostamp_flake_aligner_xytheta5x2_action");
  autostamp_flake_aligner_xytheta5x2_class flake_aligner(
      ros::this_node::getName());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  ros::spin();
  return a.exec();
}
