#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"
#ifndef __TDMMS_FINDER_HINTERFACE_H__
#define __TDMMS_FINDER_HINTERFACE_H__

extern "C" Herror tdmms_finder_Initialize();
extern "C" Herror tdmms_finder_Live();
extern "C" Herror tdmms_finder_CloseWindow();
extern "C" Herror tdmms_finder_OpenWindow();
extern "C" Herror tdmms_finder_Find(bool* dev,int *pos_x,int *pos_y, unsigned int * score);
extern "C" Herror tdmms_finder_SetParameter(char *param, Hcpar *value);
extern "C" Herror tdmms_finder_GetParameter(char *param, Hcpar *value);
extern "C" Herror tdmms_finder_hello();
extern "C" Herror tdmms_finder_SaveImage(char* filename);
extern "C" Herror tdmms_finder_SaveImage_raw(char* filename);
extern "C" Herror tdmms_finder_SaveImage_affine(char *filename);

typedef struct {
  double anisometry;
  double area;
  double area_holes;
  double bulkiness;
  double circularity;
  double column;
  double column1;
  double column2;
  double compactness;
  double connect_num;
  double contlength;
  double convexity;
  double dist_deviation;
  double dist_mean;
  double euler_number;
  double height;
  double holes_num;
  double inner_height;
  double inner_radius;
  double inner_width;
  double max_diameter;
  double moments_i1;
  double moments_i2;
  double moments_i3;
  double moments_i4;
  double moments_ia;
  double moments_ib;
  double moments_m02;
  double moments_m02_invar;
  double moments_m03;
  double moments_m03_invar;
  double moments_m11;
  double moments_m11_invar;
  double moments_m12;
  double moments_m12_invar;
  double moments_m20;
  double moments_m20_invar;
  double moments_m21;
  double moments_m21_invar;
  double moments_m30;
  double moments_m30_invar;
  double moments_phi1;
  double moments_phi2;
  double moments_psi1;
  double moments_psi2;
  double moments_psi3;
  double moments_psi4;
  double num_sides;
  double orientation;
  double outer_radius;
  double phi;
  double ra;
  double rb;
  double rect2_len1;
  double rect2_len2;
  double rect2_phi;
  double rectangularity;
  double roundness;
  double row;
  double row1;
  double row2;
  double struct_factor;
  double width;
} ObjectFeatures;


#endif
