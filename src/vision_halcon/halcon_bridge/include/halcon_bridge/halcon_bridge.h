#include "HalconCpp.h"
#include "HDevThread.h"
#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"
#include <sensor_msgs/Image.h>

namespace halcon_bridge {
using namespace HalconCpp;
void dev_display_shape_matching_results(HTuple hv_ModelID, HTuple hv_Color,
                                        HTuple hv_Row, HTuple hv_Column,
                                        HTuple hv_Angle, HTuple hv_ScaleR,
                                        HTuple hv_ScaleC, HTuple hv_Model);

void dev_open_window_fit_image(HObject ho_Image, HTuple hv_Row,
                               HTuple hv_Column, HTuple hv_WidthLimit,
                               HTuple hv_HeightLimit, HTuple *hv_WindowHandle);

void set_display_font(HTuple hv_WindowHandle, HTuple hv_Size, HTuple hv_Font,
                      HTuple hv_Bold, HTuple hv_Slant);

void disp_message(HTuple hv_WindowHandle, HTuple hv_String,
                  HTuple hv_CoordSystem, HTuple hv_Row, HTuple hv_Column,
                  HTuple hv_Color, HTuple hv_Box);
void toHObject(const sensor_msgs::Image::ConstPtr &ros_image,
               HObject *ho_image);
void toHImage(const sensor_msgs::Image::ConstPtr &ros_image,
               HImage *h_image);
void dev_update_off();

void toImageMsg(HObject ho_image, sensor_msgs::Image &ros_image);
};
