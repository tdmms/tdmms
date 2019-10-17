#ifndef __APPLE__
#include "halconcpp/HalconCpp.h"
#include "halconcpp/HDevThread.h"
#if defined(__linux__) && !defined(NO_EXPORT_APP_MAIN)
#include <X11/Xlib.h>
#endif
#else
#ifndef HC_LARGE_IMAGES
#include <HALCONCpp/HalconCpp.h>
#include <HALCONCpp/HDevThread.h>
#else
#include <HALCONCppxl/HalconCpp.h>
#include <HALCONCppxl/HDevThread.h>
#endif
#include <stdio.h>
#include <HALCON/HpThread.h>
#include <CoreFoundation/CFRunLoop.h>
#endif

#include <halcon_bridge/halcon_bridge.h>
#include "../include/tdmms_finder_hinterface.h"

using namespace HalconCpp;
using namespace halcon_bridge;
// Local iconic variables
static HObject ho_Image_BG, ho_Image_BG_R, ho_Image_BG_G;
static HObject ho_Image_BG_B, ho_Image_BG_H, ho_Image_BG_S, ho_Image_BG_V;
static HObject ho_Image, ho_Image_R, ho_Image_G, ho_Image_B, ho_Image_H;
static HObject ho_Image_S, ho_Image_V, ho_Region_H, ho_Region_S;
static HObject ho_Region_V, ho_RegionI, ho_ImageGrey, ho_ImaAmp;
static HObject ho_ImaDir, ho_RegionS, ho_RegionIII, ho_SelectedRegions;
static HObject ho_SelectedFlake;
static HObject ho_ImageGrey_8;
static HObject ho_ImageGrey_conv;
static HObject ho_Image_Sub_H, ho_Image_Sub_S, ho_Image_Sub_V;
static HObject ho_Image_tmp;
static HObject ho_Image_8, ho_ImageGauss;

// Local control variables
static HTuple hv_DeviceIdentifiers, hv_Index, hv_DeviceName;
static HTuple hv_DeviceVendor, hv_DeviceHandle, hv_AcqName, hv_AcqHandle;
static HTuple hv_exposure, hv_Width, hv_Height, hv_WindowHandle;
static HTuple hv_cent_H, hv_delta_H, hv_cent_S, hv_delta_S, hv_cent_V;
static HTuple hv_delta_V, hv_area_threshold, hv_Areas, hv_PointsY;
static HTuple hv_PointsX, hv_Number, hv_maxvalue, hv_index, hv_pointy;
static HTuple hv_pointx, hv_edge_alpha, hv_edge_low, hv_edge_high;
static HTuple hv_entropy_min, hv_entropy_max;
static HTuple hv_area_holes_threshold;
static HTuple hv_homedir, hv_bgfile, hv_bgfilepath;
static char comment[] = "Graphene Detection Program 2015.12.3 by S. Masubuchi";

extern "C" Herror tdmms_finder_Find(bool *dev, int *pos_x, int *pos_y,
                                    unsigned int *score) {
  //------------------------------------------
  // Detection Parameters
  //------------------------------------------
  hv_cent_H = 2050;
  hv_delta_H = 500;
  hv_cent_S = -1000;
  hv_delta_S = 1500;
  hv_cent_V = 1800;
  hv_delta_V = 500;
  hv_area_threshold = 500;
  hv_area_holes_threshold = 1000;
  hv_edge_alpha = 2;
  hv_edge_low = 15;
  hv_edge_high = 50;
  hv_entropy_min = 0;
  hv_entropy_max = 20;
  //------------------------------------------
  // Detection process
  //-----------------------------------------

  // Grab images from framegrabber
  GrabImage(&ho_Image, hv_AcqHandle);
  if (HDevWindowStack::IsOpen())
    DispObj(ho_Image, HDevWindowStack::GetActive());

  // Apply thresholding to grabbed image
  GaussFilter(ho_Image, &ho_ImageGauss, 5);
  Decompose3(ho_ImageGauss, &ho_Image_R, &ho_Image_G, &ho_Image_B);
  TransFromRgb(ho_Image_R, ho_Image_G, ho_Image_B, &ho_Image_H, &ho_Image_S,
               &ho_Image_V, "hsv");

  SubImage(ho_Image_H, ho_Image_BG_H, &ho_Image_Sub_H, 1, 4096/2);
  SubImage(ho_Image_S, ho_Image_BG_S, &ho_Image_Sub_S, 1, 4096/2);
  SubImage(ho_Image_V, ho_Image_BG_V, &ho_Image_Sub_V, 1, 4096/2);

  Threshold(ho_Image_Sub_H, &ho_Region_H,
            hv_cent_H - hv_delta_H + 4096/2, hv_cent_H + hv_delta_H + 4096/2);
  Threshold(ho_Image_Sub_S, &ho_Region_S,
            hv_cent_S - hv_delta_S + 4096/2, hv_cent_S + hv_delta_S + 4096/2);
  Threshold(ho_Image_Sub_V, &ho_Region_V,
            hv_cent_V - hv_delta_V + 4096/2, hv_cent_V + hv_delta_V + 4096/2);

  // Take intersections with the thresholded regions
  Intersection(ho_Region_H, ho_Region_S, &ho_RegionI);
  Intersection(ho_Region_V, ho_RegionI, &ho_RegionI);

  // Particle filter based on entropy thresholdings
  Rgb1ToGray(ho_Image, &ho_ImageGrey);
  EdgesImage(ho_ImageGrey, &ho_ImaAmp, &ho_ImaDir, "canny",
             hv_edge_alpha, "nms", hv_edge_low, hv_edge_high);
  DilationCircle(ho_ImaAmp, &ho_ImaAmp, 2);
  FillUp(ho_ImaAmp, &ho_ImaAmp);
  ErosionCircle(ho_ImaAmp, &ho_ImaAmp, 5);
  Connection(ho_ImaAmp, &ho_ImaAmp);

  BitRshift(ho_ImageGrey, &ho_ImageGrey_conv, 4);
  ConvertImageType(ho_ImageGrey_conv, &ho_ImageGrey_8, "byte");

  // Entropy filter
  SelectGray(ho_ImaAmp, ho_ImageGrey_8, &ho_RegionS, "entropy",
             "and", hv_entropy_min, hv_entropy_max);
  DilationCircle(ho_RegionS, &ho_RegionS, 2);
  Union1(ho_RegionS, &ho_RegionS);
  OpeningCircle(ho_RegionS, &ho_RegionS, 3);
  FillUp(ho_RegionS, &ho_RegionS);
  ErosionCircle(ho_RegionS, &ho_RegionS, 2);

  // Thresholding and particle filter
  Intersection(ho_RegionI, ho_RegionS, &ho_RegionIII);

  //ho_RegionIII = ho_RegionI; // Entropy Filter Off

  Connection(ho_RegionIII, &ho_RegionIII);
  ClosingCircle(ho_RegionIII, &ho_RegionIII, 3);
  ErosionCircle(ho_RegionIII, &ho_RegionIII, 5);
  DilationCircle(ho_RegionIII, &ho_RegionIII, 2);
  if (HDevWindowStack::IsOpen())
    DispObj(ho_RegionIII, HDevWindowStack::GetActive());

  SelectShape(ho_RegionIII, &ho_SelectedRegions,
              (HTuple("area").Append("area_holes")), "and",
              hv_area_threshold.TupleConcat(0),
              HTuple("max").TupleConcat(hv_area_holes_threshold));
  AreaCenter(ho_SelectedRegions, &hv_Areas, &hv_PointsY, &hv_PointsX);
  CountObj(ho_SelectedRegions, &hv_Number);

  if (0 != (hv_Number > 0)) {
    dev_update_off();
    set_display_font(hv_WindowHandle, 30, "mono", "true", "false");
    disp_message(hv_WindowHandle, "G", "window", 30, 30, "black", "true");
    TupleMax(hv_Areas, &hv_maxvalue);
    TupleFind(hv_Areas, hv_maxvalue, &hv_index);
    TupleSelect(hv_PointsY, hv_index, &hv_pointy);
    TupleSelect(hv_PointsX, hv_index, &hv_pointx);
    SelectObj(ho_SelectedRegions, &ho_SelectedFlake, hv_index + 1);

    *dev = true;

    int score_buf = hv_maxvalue;
    double pos_x_buf = hv_pointx;
    double pos_y_buf = hv_pointy;

    *pos_x = (int)pos_x_buf;
    *pos_y = (int)pos_y_buf;
    *score = (unsigned int)score_buf;
    return H_MSG_OK;
  }
  *dev = false;
  return H_MSG_OK;
}

extern "C" Herror tdmms_finder_Initialize() {
  HObject ho_Image_BG1, ho_Image_BG2, ho_Image_BG3, ho_Image_BG4;
  HObject ho_Image_BG5, ho_Image_BG6, ho_Image_BG7, ho_Image_BG8;
  HObject ho_Image_BG9;

  /* For using GPU & Open CL Comment out this part 
    QueryAvailableComputeDevices(&hv_DeviceIdentifiers);
  {
    HTuple end_val3 = (hv_DeviceIdentifiers.TupleLength()) - 1;
    HTuple step_val3 = 1;
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
  ActivateComputeDevice(hv_DeviceHandle);*/


  // Open Framegrabber (12 bit depth)
  hv_AcqName = "Pixci";
  OpenFramegrabber(hv_AcqName, 1, 1, 0, 0, 0, 0, "default", 12, "default", -1,
                   "false", "default", "default", -1, -1, &hv_AcqHandle);
  hv_exposure = 5000;
  SetFramegrabberParam(hv_AcqHandle, "exposure", hv_exposure);


  // Load background image
  TupleEnvironment("HOME", &hv_homedir);
  hv_bgfile = "/images/masubuchi/Background_Image/MoS2_85nm_20190419_12bit.tiff";
  hv_bgfilepath = hv_homedir+hv_bgfile;

  ReadImage(&ho_Image_BG, hv_bgfilepath);
  GetImageSize(ho_Image_BG, &hv_Width, &hv_Height);
  GaussFilter(ho_Image_BG, &ho_Image_BG1, 11);
  GaussFilter(ho_Image_BG1, &ho_Image_BG2, 11);
  GaussFilter(ho_Image_BG2, &ho_Image_BG3, 11);
  GaussFilter(ho_Image_BG3, &ho_Image_BG4, 11);
  GaussFilter(ho_Image_BG4, &ho_Image_BG5, 11);
  GaussFilter(ho_Image_BG5, &ho_Image_BG6, 11);
  MeanImage(ho_Image_BG6, &ho_Image_BG7, 10, 10);
  MeanImage(ho_Image_BG7, &ho_Image_BG8, 10, 10);
  Decompose3(ho_Image_BG8, &ho_Image_BG_R, &ho_Image_BG_G, &ho_Image_BG_B);
  TransFromRgb(ho_Image_BG_R, ho_Image_BG_G, ho_Image_BG_B, &ho_Image_BG_H,
               &ho_Image_BG_S, &ho_Image_BG_V, "hsv");
  return H_MSG_OK;
}

extern "C" Herror tdmms_finder_Live() {
  GrabImage(&ho_Image, hv_AcqHandle);
  DispObj(ho_Image, hv_WindowHandle);
  return H_MSG_OK;
}

extern "C" Herror tdmms_finder_CloseWindow() {
  CloseFramegrabber(hv_AcqHandle);
  CloseWindow(hv_WindowHandle);
  return H_MSG_OK;
}

extern "C" Herror tdmms_finder_OpenWindow() {
  SetWindowAttr("background_color", "black");
  OpenWindow(0, 0, hv_Width, hv_Height, 0, "", "", &hv_WindowHandle);
  SetPart(hv_WindowHandle, 0, 0, hv_Height-1, hv_Width-1);
  HDevWindowStack::Push(hv_WindowHandle);
  return H_MSG_OK;
}

extern "C" Herror tdmms_finder_GetParameter(char *param, Hcpar *value) {
  if (!strcmp(param, "comment")) {
    value->par.s = comment;
  } else if (!strcmp(param, "target_layer_num_min")) {
    value->par.l = 1;
  } else if (!strcmp(param, "target_layer_num_max")) {
    value->par.l = 1;
  }
  return H_MSG_OK;
}


extern "C" Herror tdmms_finder_ExtractFeatures(ObjectFeatures *feature) {
  HTuple hv_feature_values;
  HTuple hv_feature_values_buf;
  HTuple hv_feature_length;

  RegionFeatures(
      ho_SelectedFlake,
      ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((HTuple(
                                                                        "anisom"
                                                                        "etry")
                                                                        .Append(
                                                                             "a"
                                                                             "r"
                                                                             "e"
                                                                             "a"))
                                                                       .Append(
                                                                            "ar"
                                                                            "ea"
                                                                            "_h"
                                                                            "ol"
                                                                            "e"
                                                                            "s"))
                                                                      .Append(
                                                                           "bul"
                                                                           "kin"
                                                                           "es"
                                                                           "s"))
                                                                     .Append(
                                                                          "circ"
                                                                          "ular"
                                                                          "it"
                                                                          "y"))
                                                                    .Append(
                                                                         "colum"
                                                                         "n"))
                                                                   .Append(
                                                                        "column"
                                                                        "1"))
                                                                  .Append(
                                                                       "column"
                                                                       "2"))
                                                                 .Append(
                                                                      "compactn"
                                                                      "ess"))
                                                                .Append(
                                                                     "connect_"
                                                                     "num"))
                                                               .Append(
                                                                    "contlengt"
                                                                    "h"))
                                                              .Append(
                                                                   "convexity"))
                                                             .Append(
                                                                  "dist_"
                                                                  "deviation"))
                                                            .Append(
                                                                 "dist_mean"))
                                                           .Append(
                                                                "euler_number"))
                                                          .Append("height"))
                                                         .Append("holes_num"))
                                                        .Append("inner_height"))
                                                       .Append("inner_radius"))
                                                      .Append("inner_width"))
                                                     .Append("max_diameter"))
                                                    .Append("moments_i1"))
                                                   .Append("moments_i2"))
                                                  .Append("moments_i3"))
                                                 .Append("moments_i4"))
                                                .Append("moments_ia"))
                                               .Append("moments_ib"))
                                              .Append("moments_m02"))
                                             .Append("moments_m02_invar"))
                                            .Append("moments_m03"))
                                           .Append("moments_m03_invar"))
                                          .Append("moments_m11"))
                                         .Append("moments_m11_invar"))
                                        .Append("moments_m12"))
                                       .Append("moments_m12_invar"))
                                      .Append("moments_m20"))
                                     .Append("moments_m20_invar"))
                                    .Append("moments_m21"))
                                   .Append("moments_m21_invar"))
                                  .Append("moments_m30"))
                                 .Append("moments_m30_invar"))
                                .Append("moments_phi1")).Append("moments_phi2"))
                              .Append("moments_psi1")).Append("moments_psi2"))
                            .Append("moments_psi3")).Append("moments_psi4"))
                          .Append("num_sides")).Append("orientation"))
                        .Append("outer_radius")).Append("phi")).Append("ra"))
                     .Append("rb")).Append("rect2_len1")).Append("rect2_len2"))
                  .Append("rect2_phi")).Append("rectangularity"))
                .Append("roundness")).Append("row")).Append("row1"))
             .Append("row2")).Append("struct_factor")).Append("width")),
      &hv_feature_values);

  TupleLength(hv_feature_values, &hv_feature_length);

  if (hv_feature_length == 63) {
    TupleSelect(hv_feature_values, (const HTuple)0, &hv_feature_values_buf);
    feature->anisometry = (double)hv_feature_values_buf;

    feature->area = (double)((const HTuple &)hv_feature_values)[1];
    feature->area_holes = (double)((const HTuple &)hv_feature_values)[2];
    feature->bulkiness = (double)((const HTuple &)hv_feature_values)[3];
    feature->circularity = (double)((const HTuple &)hv_feature_values)[4];
    feature->column = (double)((const HTuple &)hv_feature_values)[5];
    feature->column1 = (double)((const HTuple &)hv_feature_values)[6];
    feature->column2 = (double)((const HTuple &)hv_feature_values)[7];
    feature->compactness = (double)((const HTuple &)hv_feature_values)[8];
    feature->connect_num = (double)((const HTuple &)hv_feature_values)[9];
    feature->contlength = (double)((const HTuple &)hv_feature_values)[10];
    feature->convexity = (double)((const HTuple &)hv_feature_values)[11];
    feature->dist_deviation = (double)((const HTuple &)hv_feature_values)[12];
    feature->dist_mean = (double)((const HTuple &)hv_feature_values)[13];
    feature->euler_number = (double)((const HTuple &)hv_feature_values)[14];
    feature->height = (double)((const HTuple &)hv_feature_values)[15];
    feature->holes_num = (double)((const HTuple &)hv_feature_values)[16];
    feature->inner_height = (double)((const HTuple &)hv_feature_values)[17];
    feature->inner_radius = (double)((const HTuple &)hv_feature_values)[18];
    feature->inner_width = (double)((const HTuple &)hv_feature_values)[19];
    feature->max_diameter = (double)((const HTuple &)hv_feature_values)[20];
    feature->moments_i1 = (double)((const HTuple &)hv_feature_values)[21];
    feature->moments_i2 = (double)((const HTuple &)hv_feature_values)[22];
    feature->moments_i3 = (double)((const HTuple &)hv_feature_values)[23];
    feature->moments_i4 = (double)((const HTuple &)hv_feature_values)[24];
    feature->moments_ia = (double)((const HTuple &)hv_feature_values)[25];
    feature->moments_ib = (double)((const HTuple &)hv_feature_values)[26];
    feature->moments_m02 = (double)((const HTuple &)hv_feature_values)[27];
    feature->moments_m02_invar =
        (double)((const HTuple &)hv_feature_values)[28];
    feature->moments_m03 = (double)((const HTuple &)hv_feature_values)[29];
    feature->moments_m03_invar =
        (double)((const HTuple &)hv_feature_values)[30];
    feature->moments_m11 = (double)((const HTuple &)hv_feature_values)[31];
    feature->moments_m11_invar =
        (double)((const HTuple &)hv_feature_values)[32];
    feature->moments_m12 = (double)((const HTuple &)hv_feature_values)[33];
    feature->moments_m12_invar =
        (double)((const HTuple &)hv_feature_values)[34];
    feature->moments_m20 = (double)((const HTuple &)hv_feature_values)[35];
    feature->moments_m20_invar =
        (double)((const HTuple &)hv_feature_values)[36];
    feature->moments_m21 = (double)((const HTuple &)hv_feature_values)[37];
    feature->moments_m21_invar =
        (double)((const HTuple &)hv_feature_values)[38];
    feature->moments_m30 = (double)((const HTuple &)hv_feature_values)[39];
    feature->moments_m30_invar =
        (double)((const HTuple &)hv_feature_values)[40];
    feature->moments_phi1 = (double)((const HTuple &)hv_feature_values)[41];
    feature->moments_phi2 = (double)((const HTuple &)hv_feature_values)[42];
    feature->moments_psi1 = (double)((const HTuple &)hv_feature_values)[43];
    feature->moments_psi2 = (double)((const HTuple &)hv_feature_values)[44];
    feature->moments_psi3 = (double)((const HTuple &)hv_feature_values)[45];
    feature->moments_psi4 = (double)((const HTuple &)hv_feature_values)[46];
    feature->num_sides = (double)((const HTuple &)hv_feature_values)[47];
    feature->orientation = (double)((const HTuple &)hv_feature_values)[48];
    feature->outer_radius = (double)((const HTuple &)hv_feature_values)[49];
    feature->phi = (double)((const HTuple &)hv_feature_values)[50];
    feature->ra = (double)((const HTuple &)hv_feature_values)[51];
    feature->rb = (double)((const HTuple &)hv_feature_values)[52];
    feature->rect2_len1 = (double)((const HTuple &)hv_feature_values)[53];
    feature->rect2_len2 = (double)((const HTuple &)hv_feature_values)[54];
    feature->rect2_phi = (double)((const HTuple &)hv_feature_values)[55];
    feature->rectangularity = (double)((const HTuple &)hv_feature_values)[56];
    feature->roundness = (double)((const HTuple &)hv_feature_values)[57];
    feature->row = (double)((const HTuple &)hv_feature_values)[58];
    feature->row1 = (double)((const HTuple &)hv_feature_values)[59];
    feature->row2 = (double)((const HTuple &)hv_feature_values)[60];
    feature->struct_factor = (double)((const HTuple &)hv_feature_values)[61];
    feature->width = (double)((const HTuple &)hv_feature_values)[62];
  }
  return H_MSG_OK;
}

extern "C" Herror tdmms_finder_SetParameter(char *param, Hcpar *value) {
  if (!strcmp(param, "exposure")) {
    hv_exposure = value->par.l;
    SetFramegrabberParam(hv_AcqHandle, "exposure", hv_exposure);
  }
  return H_MSG_OK;
}

extern "C" Herror tdmms_finder_hello() {
  printf("hello\n");
  return H_MSG_OK;
}

extern"C" Herror tdmms_finder_SaveRegion(char* filename) {
  try {
    HObject ho_RegionImage;
    RegionToBin(ho_SelectedFlake, &ho_RegionImage, 255, 0, hv_Width, hv_Height);
    WriteImage(ho_RegionImage, "jpeg", 0, filename);
    return H_MSG_OK;
  } catch (HalconCpp::HException &HDevExpDefaultException) {
    printf("Error: %d", HDevExpDefaultException.ErrorCode());
    return -1;
  }
}

extern "C" Herror tdmms_finder_SaveImage(char *filename) {
  HObject ho_Image_S, ho_Image_C;

  try {
    BitRshift(ho_Image, &ho_Image_S, 4);
    ConvertImageType(ho_Image_S, &ho_Image_C, "byte");
    WriteImage(ho_Image_C, "jpeg", 0, filename);
    return H_MSG_OK;
  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    printf("Error: %d", HDevExpDefaultException.ErrorCode());
    return -1;
  }
}

extern "C" Herror tdmms_finder_SaveImage_raw(char *filename) {
  HObject ho_Image_tmp;
  try {
    ho_Image_tmp = ho_Image;
    WriteImage(ho_Image_tmp, "tiff lzw", 0, filename);
    return H_MSG_OK;
  } catch (HalconCpp::HException &HDevExpDefaultException) {
    printf("Error in SaveRawImage: %d\n", HDevExpDefaultException.ErrorCode());
    return -1;
  }
}
extern "C" Herror tdmms_finder_SaveImage_affine(char *filename) {
  HTuple hv_HomMat2DIdentity, hv_HomMat2DScale0125;
  HObject ho_GrayImage, ho_GrayImageAffinTrans;
  HObject ho_GrayImage_shift;

  HomMat2dIdentity(&hv_HomMat2DIdentity);
  HomMat2dScale(hv_HomMat2DIdentity, 0.125, 0.125, 0, 0, &hv_HomMat2DScale0125);
  Rgb1ToGray(ho_Image, &ho_GrayImage);
  AffineTransImage(ho_GrayImage, &ho_GrayImageAffinTrans, hv_HomMat2DScale0125,
                   "nearest_neighbor", "true");
  BitRshift(ho_GrayImageAffinTrans, &ho_GrayImage_shift, 4);
  WriteImage(ho_GrayImage_shift, "tiff", 0, filename);
  return H_MSG_OK;
}
