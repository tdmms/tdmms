// Copyright 2016 by S. Masubuchi
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>

#include <actionlib/server/simple_action_server.h>
#include <tdmms_autostamp_flake_positioner_action_fast/PositionFlakeFastAction.h>
#include <stamper_sample_xy/velocity.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <halcon_bridge/halcon_bridge.h>
#include <QtSql>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"

using namespace HalconCpp;
class autostamp_flake_positioner_fast_class {
 protected:
  ros::NodeHandle node_;
  ros::Publisher stamper_sample_xy_cmd_stp;
  ros::Publisher stamper_nic_100a_cmd_inten;
  ros::Publisher stamper_nikon_revolv_set;
  ros::Subscriber image_subscriber_main;
  ros::ServiceClient stamper_sample_xy_wait_for_stop_client;
  QSqlDatabase db;

  actionlib::SimpleActionServer<
      tdmms_autostamp_flake_positioner_action_fast::PositionFlakeFastAction>
      as_;
  std::string action_name_;
  tdmms_autostamp_flake_positioner_action_fast::PositionFlakeFastFeedback
      feedback_;
  tdmms_autostamp_flake_positioner_action_fast::PositionFlakeFastResult result_;
  HTuple hv_AcqHandle, hv_DeviceHandle, hv_Index, hv_DeviceName;
  HTuple hv_DeviceVendor;
  HObject ho_Image, ho_Image_buf;
  HTuple hv_WindowHandle_main, hv_WindowHandle_sub, hv_DeviceIdentifiers;
  double edge_detection_pixel_to_stp_factor;

 public:
  explicit autostamp_flake_positioner_fast_class(std::string name)
      : as_(node_, name,
            boost::bind(
                &autostamp_flake_positioner_fast_class::executePositionFlake,
                this, _1),
            false),
        action_name_(name) {

    /////////////////////////////////////////
    /// Activate GPU
    ////////////////////////////////////////
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

    /*****************************************************************************
     ** Initialize Publisher Nodes
     *****************************************************************************/
    stamper_sample_xy_cmd_stp = node_.advertise<geometry_msgs::Point>(
        "/stamper_sample_xy_master/cmd_stp_move", 1);
    stamper_nikon_revolv_set = node_.advertise<std_msgs::UInt8>(
        "/stamper_nikon_master/cmd_revolv_set", 1);
    stamper_nic_100a_cmd_inten =
        node_.advertise<std_msgs::UInt8>("/stamper_nic_100a/cmd_inten", 1);
    /*****************************************************************************
     ** Initialize Service Client
     *****************************************************************************/
    stamper_sample_xy_wait_for_stop_client =
        node_.serviceClient<std_srvs::Empty>(
            "/stamper_sample_xy_master/wait_for_stop");

    /*****************************************************************************
     ** Load Parameters from parameter server
     *****************************************************************************/
    std::string parameter_name;
    parameter_name =
        "/tdmms_stamper_autostamp/edge_detection_pixel_to_stp_factor";
    if (!node_.getParam(parameter_name, edge_detection_pixel_to_stp_factor)) {
      ROS_ERROR("Could not retrieve parameter %s in namespace %s.",
                parameter_name.c_str(), node_.getNamespace().c_str());
      exit(-1);
    }

    /*****************************************************************************
     ** Connecting to MySQL Database
     *****************************************************************************/
    QSqlError err;
    if (QSqlDatabase::drivers().isEmpty()) {
      ROS_ERROR(
          "No database drivers found"
          "This program requires at least one Qt database driver. "
          "Please check the documentation how to build the "
          "Qt SQL plugins.");
    }

    db = QSqlDatabase::addDatabase(QString("QMYSQL"),
                                   QString("Browser%1").arg(1));
    db.setDatabaseName(QString("mydb"));
    db.setHostName(QString("localhost"));
    db.setPort(3306);
    if (!db.open(QString("root"), QString("mlab2dmms"))) {
      err = db.lastError();
      db = QSqlDatabase();
      QSqlDatabase::removeDatabase(QString("Browser%1").arg(1));
    } else {
      ROS_DEBUG("connected to MYSQL Server \n");
    }

    as_.start();
  }

  ~autostamp_flake_positioner_fast_class(void) { db.close(); }

  void imageStreamCallback(const sensor_msgs::Image::ConstPtr &Image) {
    ////////////////////////////////////////
    /// Callback function for image stream
    ////////////////////////////////////////
    halcon_bridge::toHObject(Image, &ho_Image_buf);
    ho_Image = ho_Image_buf;
  }

  void executePositionFlake(const tdmms_autostamp_flake_positioner_action_fast::
                                PositionFlakeFastGoalConstPtr &goalparam) {
    bool success = true;
    //    as_.acceptNewGoal();

    ////////////////////////////////////////
    /// Open Output Windows
    ////////////////////////////////////////
    std::string initfname_small;

    initfname_small = getenv("HOME");
    initfname_small = initfname_small + "/img744x480.jpg";
    ReadImage(&ho_Image, initfname_small.c_str());
    HalconCpp::SetWindowAttr("background_color", "black");
    HalconCpp::SetWindowAttr("window_title", "Optical Microscope");
    HalconCpp::OpenWindow(0, 744 * 1.5, 744 * 1.5, 480 * 1.5, 0, "", "",
                          &hv_WindowHandle_main);
    HalconCpp::HDevWindowStack::Push(hv_WindowHandle_main);
    HalconCpp::SetPart(hv_WindowHandle_main, 0, 0, 480, 744);
    halcon_bridge::set_display_font(hv_WindowHandle_main, 16, "mono", "true",
                                    "false");
    std::string initfname_large;

    initfname_large = getenv("HOME");
    initfname_large = initfname_small + "/img2040x1086.jpg";
    ReadImage(&ho_Image, initfname_large.c_str());
    HalconCpp::SetWindowAttr("window_title", "Template Image");
    HalconCpp::OpenWindow(0, 2560, 744 * 1.5, 480 * 1.5, 0, "", "",
                          &hv_WindowHandle_sub);
    HalconCpp::HDevWindowStack::Push(hv_WindowHandle_sub);
    HalconCpp::SetPart(hv_WindowHandle_sub, 0, 0, 1086, 2040);
    halcon_bridge::set_display_font(hv_WindowHandle_sub, 16, "mono", "true",
                                    "false");

    ////////////////////////////////////////
    /// Subscribe to Image Stream
    ////////////////////////////////////////
    image_subscriber_main = node_.subscribe(
        "/autostamp_camera_streamer/image_main", 100,
        &autostamp_flake_positioner_fast_class::imageStreamCallback, this);
    ros::Duration(1).sleep();

    ////////////////////////////////////////
    /// Setting Tartlet Position to 5x
    ////////////////////////////////////////
    std_msgs::UInt8 pos;
    pos.data = 1;
    stamper_nikon_revolv_set.publish(pos);
    ros::Duration(0.1).sleep();

    //////////////////////////////////////
    //// Setting Illumination Intensity to 5
    ///////////////////////////////////////
    std_msgs::UInt8 light_inten;
    light_inten.data = 5;
    stamper_nic_100a_cmd_inten.publish(light_inten);
    ros::Duration(0.1).sleep();

    geometry_msgs::Point point;
    std_srvs::Empty emp_srv;

    ///////////////
    // Local iconic variables
    ///////////////
    HObject ho_ModelRegion, ho_TemplateImage;
    HObject ho_Image_Match;

    ////////////////
    // Local control variables
    ////////////////
    HTuple hv_ModelId, hv_pix_to_step_factor;
    HTuple hv_Width, hv_Height;
    HTuple hv_deltax, hv_deltay, hv_Seconds1, hv_ModelRow;
    HTuple hv_ModelColumn, hv_ModelAngle, hv_ModelScale, hv_ModelScore;
    HTuple hv_Seconds2, hv_movevector_x, hv_movevector_y, hv_rotangle;
    HTuple hv_r_movevector_x, hv_r_movevector_y;
    HTuple hv_r_movevector_x_c, hv_r_movevector_y_c;

    std::vector<int> id_chip;
    std::vector<double> point_x_alignment;
    std::vector<double> point_y_alignment;
    QStringList filename_alignment;

    SetSystem("border_shape_models", "true");

    /////////////////////////////
    //  Intialize Halcon Parameters
    //////////////////////////////
    hv_pix_to_step_factor = edge_detection_pixel_to_stp_factor;

    //////////////////////////////////////////
    /// Extract Target Position
    /////////////////////////////////////////
    QSqlQuery q("", db);
    q.prepare(
        "select position_x, position_y from flake_images where "
        "filename_flake_image = ?");
    q.addBindValue(goalparam->filename_target.data.c_str());
    if (!q.exec()) {
      ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                q.lastError().text().toStdString().c_str(),
                q.lastQuery().toStdString().c_str());
      HDevWindowStack::SetActive(hv_WindowHandle_sub);
      if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
      HDevWindowStack::SetActive(hv_WindowHandle_main);
      if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
      image_subscriber_main.shutdown();
      success = false;
      as_.setAborted();
      return;
    } else {
      ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
    }
    q.next();
    double point_x_target = static_cast<double>(q.value(0).toInt());
    double point_y_target = static_cast<double>(q.value(1).toInt());

    //////////////////////////////////////////
    /// Extract Alignment Images
    /////////////////////////////////////////
    q.prepare("CALL select_alignment_image(:filename)");
    q.bindValue(":filename", goalparam->filename_target.data.c_str());
    if (!q.exec()) {
      ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                q.lastError().text().toStdString().c_str(),
                q.lastQuery().toStdString().c_str());
      HDevWindowStack::SetActive(hv_WindowHandle_sub);
      if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
      HDevWindowStack::SetActive(hv_WindowHandle_main);
      if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
      image_subscriber_main.shutdown();
      success = false;
      as_.setAborted();
      return;
    } else {
      ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
    }
    q.prepare("select * from mydb.temp");
    if (!q.exec()) {
      ROS_ERROR("QUERY ERROR, ERROR:%s SENT:%s",
                q.lastError().text().toStdString().c_str(),
                q.lastQuery().toStdString().c_str());
      HDevWindowStack::SetActive(hv_WindowHandle_sub);
      if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
      HDevWindowStack::SetActive(hv_WindowHandle_main);
      if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
      image_subscriber_main.shutdown();
      success = false;
      as_.setAborted();
      return;
    } else {
      ROS_DEBUG("QUERY SUCCESS, SENT:%s", q.lastQuery().toStdString().c_str());
    }
    while (q.next()) {
      id_chip.push_back(q.value(0).toInt());
      point_x_alignment.push_back(static_cast<double>(q.value(1).toInt()));
      point_y_alignment.push_back(static_cast<double>(q.value(2).toInt()));
      filename_alignment << q.value(3).toString();
      ROS_INFO("%s", q.value(3).toString().toStdString().c_str());
    }

    /////////////////////////////////////////////
    ////  Main Loop: Searchi
    ////////////////////////////////////////////
    HDevWindowStack::SetActive(hv_WindowHandle_main);
    if (HDevWindowStack::IsOpen())
      DispObj(ho_Image, HDevWindowStack::GetActive());

    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    SetSystem("border_shape_models", "true");
    GenRectangle1(&ho_ModelRegion, 0, 0, 475.5, 736.5);
    ReduceDomain(ho_Image, ho_ModelRegion, &ho_TemplateImage);

    /////////////////////////////////////////////////
    //// Create Scaled Shape Model
    ////////////////////////////////////////////////
    try {
      CreateScaledShapeModel(
          ho_TemplateImage, 4, HTuple(0).TupleRad(), HTuple(360).TupleRad(),
          HTuple(0.1525).TupleRad(), 1.05, 1.1, 0.0027,
          (HTuple("none").Append("no_pregeneration")), "use_polarity",
          ((HTuple(11).Append(16)).Append(4)), 4, &hv_ModelId);
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

    SetShapeModelParam(hv_ModelId, "timeout", 10000);

    for (int i = 0; i < filename_alignment.size(); ++i) {
      ReadImage(&ho_Image_Match,
                HTuple(filename_alignment.at(i).toStdString().c_str()));

      HDevWindowStack::SetActive(hv_WindowHandle_sub);
      if (HDevWindowStack::IsOpen())
        DispObj(ho_Image_Match, HDevWindowStack::GetActive());

      GetImageSize(ho_Image_Match, &hv_Width, &hv_Height);
      CountSeconds(&hv_Seconds1);
      FindScaledShapeModel(ho_Image_Match, hv_ModelId, HTuple(0).TupleRad(),
                           HTuple(360).TupleRad(), 1.05, 1.1, 0.4, 1, 0.5,
                           "least_squares", (HTuple(4).Append(1)), 0,
                           &hv_ModelRow, &hv_ModelColumn, &hv_ModelAngle,
                           &hv_ModelScale, &hv_ModelScore);
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
      if (0 != ((hv_ModelScore.TupleLength()) != 0)) {
        hv_deltax =
            ((-(hv_ModelColumn - (hv_Width / 2))) * hv_pix_to_step_factor) / 20;
        hv_deltay =
            ((-(hv_ModelRow - (hv_Height / 2))) * hv_pix_to_step_factor) / 20;

        hv_movevector_x = (point_x_target - point_x_alignment.at(i) +
                           static_cast<double>(hv_deltax) / 100);
        hv_movevector_y = (point_y_target - point_y_alignment.at(i) +
                           static_cast<double>(hv_deltay) / 100);
        hv_rotangle = -(hv_ModelAngle + HTuple(3).TupleRad());

        hv_r_movevector_x = ((hv_rotangle.TupleCos()) * hv_movevector_x) -
                            ((hv_rotangle.TupleSin()) * hv_movevector_y);
        hv_r_movevector_y = ((hv_rotangle.TupleSin()) * hv_movevector_x) +
                            ((hv_rotangle.TupleCos()) * hv_movevector_y);

        TupleCeil(hv_r_movevector_x, &hv_r_movevector_x_c);
        TupleCeil(hv_r_movevector_y, &hv_r_movevector_y_c);
        if (0 != (hv_r_movevector_x_c == -0.0)) {
          hv_r_movevector_x_c = 0.0;
        }
        if (0 != (hv_r_movevector_y_c == 0.0)) {
          hv_r_movevector_y_c = 0.0;
        }

        point.x = (-1) * static_cast<double>(hv_r_movevector_x_c) * 100;
        point.y = (1) * static_cast<double>(hv_r_movevector_y_c) * 100;

        stamper_sample_xy_cmd_stp.publish(point);
        stamper_sample_xy_wait_for_stop_client.call(emp_srv);

        HDevWindowStack::SetActive(hv_WindowHandle_sub);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        HDevWindowStack::SetActive(hv_WindowHandle_main);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        image_subscriber_main.shutdown();
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        success = true;
        as_.setSucceeded(result_);
        return;
      }
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        success = false;
        as_.setPreempted();
        HDevWindowStack::SetActive(hv_WindowHandle_sub);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        HDevWindowStack::SetActive(hv_WindowHandle_main);
        if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
        image_subscriber_main.shutdown();
        return;
      }
    }
    HDevWindowStack::SetActive(hv_WindowHandle_sub);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    HDevWindowStack::SetActive(hv_WindowHandle_main);
    if (HDevWindowStack::IsOpen()) CloseWindow(HDevWindowStack::Pop());
    image_subscriber_main.shutdown();
    as_.setAborted();
    success = false;
    ROS_ERROR("No match could be found");
    return;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tdmms_autostamp_flake_positioner_action_fast");
  autostamp_flake_positioner_fast_class flake_positioner(
      ros::this_node::getName());
    ros::spin();
    return 0;
}
