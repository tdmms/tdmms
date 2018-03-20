// Copyright 2016.4 by S. Masubuchi

#include "Halcon.h"
#include "hlib/CIOFrameGrab.h"
#include "hlib/HInstance.h"
#include "hlib/HpThread.h"
#include <sensor_msgs/Image.h>

#include "../../include/halcon_bridge/halcon_bridge.h"

namespace halcon_bridge {
using namespace HalconCpp;
void dev_display_shape_matching_results(HTuple hv_ModelID, HTuple hv_Color,
                                        HTuple hv_Row, HTuple hv_Column,
                                        HTuple hv_Angle, HTuple hv_ScaleR,
                                        HTuple hv_ScaleC, HTuple hv_Model) {
  // Local iconic variables
  HObject ho_ModelContours, ho_ContoursAffinTrans;

  // Local control variables
  HTuple hv_NumMatches, hv_Index, hv_Match, hv_HomMat2DIdentity;
  HTuple hv_HomMat2DScale, hv_HomMat2DRotate, hv_HomMat2DTranslate;
  hv_NumMatches = hv_Row.TupleLength();
  if (0 != (hv_NumMatches > 0)) {
    if (0 != ((hv_ScaleR.TupleLength()) == 1)) {
      TupleGenConst(hv_NumMatches, hv_ScaleR, &hv_ScaleR);
    }
    if (0 != ((hv_ScaleC.TupleLength()) == 1)) {
      TupleGenConst(hv_NumMatches, hv_ScaleC, &hv_ScaleC);
    }
    if (0 != ((hv_Model.TupleLength()) == 0)) {
      TupleGenConst(hv_NumMatches, 0, &hv_Model);
    } else if (0 != ((hv_Model.TupleLength()) == 1)) {
      TupleGenConst(hv_NumMatches, hv_Model, &hv_Model);
    }

    {
      HTuple end_val15 = (hv_ModelID.TupleLength()) - 1;
      HTuple step_val15 = 1;
      for (hv_Index = 0; hv_Index.Continue(end_val15, step_val15);
           hv_Index += step_val15) {
        GetShapeModelContours(&ho_ModelContours, HTuple(hv_ModelID[hv_Index]),
                              1);
        if (HDevWindowStack::IsOpen())
          SetColor(HDevWindowStack::GetActive(),
                   HTuple(hv_Color[hv_Index % (hv_Color.TupleLength())]));
        {
          HTuple end_val18 = hv_NumMatches - 1;
          HTuple step_val18 = 1;
          for (hv_Match = 0; hv_Match.Continue(end_val18, step_val18);
               hv_Match += step_val18) {
            if (0 != (hv_Index == HTuple(hv_Model[hv_Match]))) {
              HomMat2dIdentity(&hv_HomMat2DIdentity);
              HomMat2dScale(hv_HomMat2DIdentity, HTuple(hv_ScaleR[hv_Match]),
                            HTuple(hv_ScaleC[hv_Match]), 0, 0,
                            &hv_HomMat2DScale);
              HomMat2dRotate(hv_HomMat2DScale, HTuple(hv_Angle[hv_Match]), 0, 0,
                             &hv_HomMat2DRotate);
              HomMat2dTranslate(hv_HomMat2DRotate, HTuple(hv_Row[hv_Match]),
                                HTuple(hv_Column[hv_Match]),
                                &hv_HomMat2DTranslate);
              AffineTransContourXld(ho_ModelContours, &ho_ContoursAffinTrans,
                                    hv_HomMat2DTranslate);
              if (HDevWindowStack::IsOpen())
                DispObj(ho_ContoursAffinTrans, HDevWindowStack::GetActive());
            }
          }
        }
      }
    }
  }
  return;
}

void disp_message(HTuple hv_WindowHandle, HTuple hv_String,
                  HTuple hv_CoordSystem, HTuple hv_Row, HTuple hv_Column,
                  HTuple hv_Color, HTuple hv_Box) {
  HTuple hv_Red, hv_Green, hv_Blue, hv_Row1Part;
  HTuple hv_Column1Part, hv_Row2Part, hv_Column2Part, hv_RowWin;
  HTuple hv_ColumnWin, hv_WidthWin, hv_HeightWin, hv_MaxAscent;
  HTuple hv_MaxDescent, hv_MaxWidth, hv_MaxHeight, hv_R1;
  HTuple hv_C1, hv_FactorRow, hv_FactorColumn, hv_UseShadow;
  HTuple hv_ShadowColor, hv_Exception, hv_Width, hv_Index;
  HTuple hv_Ascent, hv_Descent, hv_W, hv_H, hv_FrameHeight;
  HTuple hv_FrameWidth, hv_R2, hv_C2, hv_DrawMode, hv_CurrentColor;

  GetRgb(hv_WindowHandle, &hv_Red, &hv_Green, &hv_Blue);
  GetPart(hv_WindowHandle, &hv_Row1Part, &hv_Column1Part, &hv_Row2Part,
          &hv_Column2Part);
  GetWindowExtents(hv_WindowHandle, &hv_RowWin, &hv_ColumnWin, &hv_WidthWin,
                   &hv_HeightWin);
  SetPart(hv_WindowHandle, 0, 0, hv_HeightWin - 1, hv_WidthWin - 1);

  if (0 != (hv_Row == -1)) {
    hv_Row = 12;
  }
  if (0 != (hv_Column == -1)) {
    hv_Column = 12;
  }
  if (0 != (hv_Color == HTuple())) {
    hv_Color = "";
  }
  //
  hv_String = (("" + hv_String) + "").TupleSplit("\n");
  GetFontExtents(hv_WindowHandle, &hv_MaxAscent, &hv_MaxDescent, &hv_MaxWidth,
                 &hv_MaxHeight);
  if (0 != (hv_CoordSystem == HTuple("window"))) {
    hv_R1 = hv_Row;
    hv_C1 = hv_Column;
  } else {
    // Transform image to window coordinates
    hv_FactorRow = (1. * hv_HeightWin) / ((hv_Row2Part - hv_Row1Part) + 1);
    hv_FactorColumn =
        (1. * hv_WidthWin) / ((hv_Column2Part - hv_Column1Part) + 1);
    hv_R1 = ((hv_Row - hv_Row1Part) + 0.5) * hv_FactorRow;
    hv_C1 = ((hv_Column - hv_Column1Part) + 0.5) * hv_FactorColumn;
  }
  //
  // Display text box depending on text size
  hv_UseShadow = 1;
  hv_ShadowColor = "gray";
  if (0 != (HTuple(hv_Box[0]) == HTuple("true"))) {
    hv_Box[0] = "#fce9d4";
    hv_ShadowColor = "#f28d26";
  }
  if (0 != ((hv_Box.TupleLength()) > 1)) {
    if (0 != (HTuple(hv_Box[1]) == HTuple("true"))) {
      // Use default ShadowColor set above
    } else if (0 != (HTuple(hv_Box[1]) == HTuple("false"))) {
      hv_UseShadow = 0;
    } else {
      hv_ShadowColor = ((const HTuple &)hv_Box)[1];
      // Valid color?
      try {
        SetColor(hv_WindowHandle, HTuple(hv_Box[1]));
      }
      // catch (Exception)
      catch (HalconCpp::HException &HDevExpDefaultException) {
        HDevExpDefaultException.ToHTuple(&hv_Exception);
        hv_Exception =
            "Wrong value of control parameter Box[1] (must be a 'true', "
            "'false', or a valid color string)";
        throw HalconCpp::HException(hv_Exception);
      }
    }
  }
  if (0 != (HTuple(hv_Box[0]) != HTuple("false"))) {
    // Valid color?
    try {
      SetColor(hv_WindowHandle, HTuple(hv_Box[0]));
    }
    // catch (Exception)
    catch (HalconCpp::HException &HDevExpDefaultException) {
      HDevExpDefaultException.ToHTuple(&hv_Exception);
      hv_Exception =
          "Wrong value of control parameter Box[0] (must be a 'true', 'false', "
          "or a valid color string)";
      throw HalconCpp::HException(hv_Exception);
    }
    // Calculate box extents
    hv_String = (" " + hv_String) + " ";
    hv_Width = HTuple();
    {
      HTuple end_val93 = (hv_String.TupleLength()) - 1;
      HTuple step_val93 = 1;
      for (hv_Index = 0; hv_Index.Continue(end_val93, step_val93);
           hv_Index += step_val93) {
        GetStringExtents(hv_WindowHandle, HTuple(hv_String[hv_Index]),
                         &hv_Ascent, &hv_Descent, &hv_W, &hv_H);
        hv_Width = hv_Width.TupleConcat(hv_W);
      }
    }
    hv_FrameHeight = hv_MaxHeight * (hv_String.TupleLength());
    hv_FrameWidth = (HTuple(0).TupleConcat(hv_Width)).TupleMax();
    hv_R2 = hv_R1 + hv_FrameHeight;
    hv_C2 = hv_C1 + hv_FrameWidth;
    // Display rectangles
    GetDraw(hv_WindowHandle, &hv_DrawMode);
    SetDraw(hv_WindowHandle, "fill");
    // Set shadow color
    SetColor(hv_WindowHandle, hv_ShadowColor);
    if (0 != hv_UseShadow) {
      DispRectangle1(hv_WindowHandle, hv_R1 + 1, hv_C1 + 1, hv_R2 + 1,
                     hv_C2 + 1);
    }
    // Set box color
    SetColor(hv_WindowHandle, HTuple(hv_Box[0]));
    DispRectangle1(hv_WindowHandle, hv_R1, hv_C1, hv_R2, hv_C2);
    SetDraw(hv_WindowHandle, hv_DrawMode);
  }
  // Write text.
  {
    HTuple end_val115 = (hv_String.TupleLength()) - 1;
    HTuple step_val115 = 1;
    for (hv_Index = 0; hv_Index.Continue(end_val115, step_val115);
         hv_Index += step_val115) {
      hv_CurrentColor =
          ((const HTuple &)hv_Color)[hv_Index % (hv_Color.TupleLength())];
      if (0 != (HTuple(hv_CurrentColor != HTuple(""))
                    .TupleAnd(hv_CurrentColor != HTuple("auto")))) {
        SetColor(hv_WindowHandle, hv_CurrentColor);
      } else {
        SetRgb(hv_WindowHandle, hv_Red, hv_Green, hv_Blue);
      }
      hv_Row = hv_R1 + (hv_MaxHeight * hv_Index);
      SetTposition(hv_WindowHandle, hv_Row, hv_C1);
      WriteString(hv_WindowHandle, HTuple(hv_String[hv_Index]));
    }
  }
  // Reset changed window settings
  SetRgb(hv_WindowHandle, hv_Red, hv_Green, hv_Blue);
  SetPart(hv_WindowHandle, hv_Row1Part, hv_Column1Part, hv_Row2Part,
          hv_Column2Part);
  return;
}

// Chapter: Graphics / Text
// Short Description: Set font independent of OS
void set_display_font(HTuple hv_WindowHandle, HTuple hv_Size, HTuple hv_Font,
                      HTuple hv_Bold, HTuple hv_Slant) {
  // Local iconic variables

  // Local control variables
  HTuple hv_OS, hv_BufferWindowHandle, hv_Ascent;
  HTuple hv_Descent, hv_Width, hv_Height, hv_Scale, hv_Exception;
  HTuple hv_SubFamily, hv_Fonts, hv_SystemFonts, hv_Guess;
  HTuple hv_I, hv_Index, hv_AllowedFontSizes, hv_Distances;
  HTuple hv_Indices, hv_FontSelRegexp, hv_FontsCourier;

  // This procedure sets the text font of the current window with
  // the specified attributes.
  // It is assumed that following fonts are installed on the system:
  // Windows: Courier New, Arial Times New Roman
  // Mac OS X: CourierNewPS, Arial, TimesNewRomanPS
  // Linux: courier, helvetica, times
  // Because fonts are displayed smaller on Linux than on Windows,
  // a scaling factor of 1.25 is used the get comparable results.
  // For Linux, only a limited number of font sizes is supported,
  // to get comparable results, it is recommended to use one of the
  // following sizes: 9, 11, 14, 16, 20, 27
  //
  // Input parameters:
  // WindowHandle: The graphics window for which the font will be set
  // Size: The font size. If Size=-1, the default of 16 is used.
  // Bold: If set to 'true', a bold font is used
  // Slant: If set to 'true', a slanted font is used
  //
  GetSystem("operating_system", &hv_OS);
  // dev_get_preferences(...); only in hdevelop
  // dev_set_preferences(...); only in hdevelop
  if (0 != (HTuple(hv_Size == HTuple()).TupleOr(hv_Size == -1))) {
    hv_Size = 16;
  }
  if (0 != ((hv_OS.TupleSubstr(0, 2)) == HTuple("Win"))) {
    // Set font on Windows systems
    try {
      // Check, if font scaling is switched on
      OpenWindow(0, 0, 256, 256, 0, "buffer", "", &hv_BufferWindowHandle);
      SetFont(hv_BufferWindowHandle, "-Consolas-16-*-0-*-*-1-");
      GetStringExtents(hv_BufferWindowHandle, "test_string", &hv_Ascent,
                       &hv_Descent, &hv_Width, &hv_Height);
      // Expected width is 110
      hv_Scale = 110.0 / hv_Width;
      hv_Size = (hv_Size * hv_Scale).TupleInt();
      CloseWindow(hv_BufferWindowHandle);
    }
    // catch (Exception)
    catch (HalconCpp::HException &HDevExpDefaultException) {
      HDevExpDefaultException.ToHTuple(&hv_Exception);
      // throw (Exception)
    }
    if (0 != (HTuple(hv_Font == HTuple("Courier"))
                  .TupleOr(hv_Font == HTuple("courier")))) {
      hv_Font = "Courier New";
    } else if (0 != (hv_Font == HTuple("mono"))) {
      hv_Font = "Consolas";
    } else if (0 != (hv_Font == HTuple("sans"))) {
      hv_Font = "Arial";
    } else if (0 != (hv_Font == HTuple("serif"))) {
      hv_Font = "Times New Roman";
    }
    if (0 != (hv_Bold == HTuple("true"))) {
      hv_Bold = 1;
    } else if (0 != (hv_Bold == HTuple("false"))) {
      hv_Bold = 0;
    } else {
      hv_Exception = "Wrong value of control parameter Bold";
      throw HalconCpp::HException(hv_Exception);
    }
    if (0 != (hv_Slant == HTuple("true"))) {
      hv_Slant = 1;
    } else if (0 != (hv_Slant == HTuple("false"))) {
      hv_Slant = 0;
    } else {
      hv_Exception = "Wrong value of control parameter Slant";
      throw HalconCpp::HException(hv_Exception);
    }
    try {
      SetFont(hv_WindowHandle,
              ((((((("-" + hv_Font) + "-") + hv_Size) + "-*-") + hv_Slant) +
                "-*-*-") +
               hv_Bold) +
                  "-");
    }
    // catch (Exception)
    catch (HalconCpp::HException &HDevExpDefaultException) {
      HDevExpDefaultException.ToHTuple(&hv_Exception);
      // throw (Exception)
    }
  } else if (0 != ((hv_OS.TupleSubstr(0, 2)) == HTuple("Dar"))) {
    // Set font on Mac OS X systems. Since OS X does not have a strict naming
    // scheme for font attributes, we use tables to determine the correct font
    // name.
    hv_SubFamily = 0;
    if (0 != (hv_Slant == HTuple("true"))) {
      hv_SubFamily = hv_SubFamily | 1;
    } else if (0 != (hv_Slant != HTuple("false"))) {
      hv_Exception = "Wrong value of control parameter Slant";
      throw HalconCpp::HException(hv_Exception);
    }
    if (0 != (hv_Bold == HTuple("true"))) {
      hv_SubFamily = hv_SubFamily | 2;
    } else if (0 != (hv_Bold != HTuple("false"))) {
      hv_Exception = "Wrong value of control parameter Bold";
      throw HalconCpp::HException(hv_Exception);
    }
    if (0 != (hv_Font == HTuple("mono"))) {
      hv_Fonts.Clear();
      hv_Fonts[0] = "Menlo-Regular";
      hv_Fonts[1] = "Menlo-Italic";
      hv_Fonts[2] = "Menlo-Bold";
      hv_Fonts[3] = "Menlo-BoldItalic";
    } else if (0 != (HTuple(hv_Font == HTuple("Courier"))
                         .TupleOr(hv_Font == HTuple("courier")))) {
      hv_Fonts.Clear();
      hv_Fonts[0] = "CourierNewPSMT";
      hv_Fonts[1] = "CourierNewPS-ItalicMT";
      hv_Fonts[2] = "CourierNewPS-BoldMT";
      hv_Fonts[3] = "CourierNewPS-BoldItalicMT";
    } else if (0 != (hv_Font == HTuple("sans"))) {
      hv_Fonts.Clear();
      hv_Fonts[0] = "ArialMT";
      hv_Fonts[1] = "Arial-ItalicMT";
      hv_Fonts[2] = "Arial-BoldMT";
      hv_Fonts[3] = "Arial-BoldItalicMT";
    } else if (0 != (hv_Font == HTuple("serif"))) {
      hv_Fonts.Clear();
      hv_Fonts[0] = "TimesNewRomanPSMT";
      hv_Fonts[1] = "TimesNewRomanPS-ItalicMT";
      hv_Fonts[2] = "TimesNewRomanPS-BoldMT";
      hv_Fonts[3] = "TimesNewRomanPS-BoldItalicMT";
    } else {
      // Attempt to figure out which of the fonts installed on the system
      // the user could have meant.
      QueryFont(hv_WindowHandle, &hv_SystemFonts);
      hv_Fonts.Clear();
      hv_Fonts.Append(hv_Font);
      hv_Fonts.Append(hv_Font);
      hv_Fonts.Append(hv_Font);
      hv_Fonts.Append(hv_Font);
      hv_Guess.Clear();
      hv_Guess.Append(hv_Font);
      hv_Guess.Append(hv_Font + "-Regular");
      hv_Guess.Append(hv_Font + "MT");
      {
        HTuple end_val100 = (hv_Guess.TupleLength()) - 1;
        HTuple step_val100 = 1;
        for (hv_I = 0; hv_I.Continue(end_val100, step_val100);
             hv_I += step_val100) {
          TupleFind(hv_SystemFonts, HTuple(hv_Guess[hv_I]), &hv_Index);
          if (0 != (hv_Index != -1)) {
            hv_Fonts[0] = HTuple(hv_Guess[hv_I]);
            break;
          }
        }
      }
      // Guess name of slanted font
      hv_Guess.Clear();
      hv_Guess.Append(hv_Font + "-Italic");
      hv_Guess.Append(hv_Font + "-ItalicMT");
      hv_Guess.Append(hv_Font + "-Oblique");
      {
        HTuple end_val109 = (hv_Guess.TupleLength()) - 1;
        HTuple step_val109 = 1;
        for (hv_I = 0; hv_I.Continue(end_val109, step_val109);
             hv_I += step_val109) {
          TupleFind(hv_SystemFonts, HTuple(hv_Guess[hv_I]), &hv_Index);
          if (0 != (hv_Index != -1)) {
            hv_Fonts[1] = HTuple(hv_Guess[hv_I]);
            break;
          }
        }
      }
      // Guess name of bold font
      hv_Guess.Clear();
      hv_Guess.Append(hv_Font + "-Bold");
      hv_Guess.Append(hv_Font + "-BoldMT");
      {
        HTuple end_val118 = (hv_Guess.TupleLength()) - 1;
        HTuple step_val118 = 1;
        for (hv_I = 0; hv_I.Continue(end_val118, step_val118);
             hv_I += step_val118) {
          TupleFind(hv_SystemFonts, HTuple(hv_Guess[hv_I]), &hv_Index);
          if (0 != (hv_Index != -1)) {
            hv_Fonts[2] = HTuple(hv_Guess[hv_I]);
            break;
          }
        }
      }
      // Guess name of bold slanted font
      hv_Guess.Clear();
      hv_Guess.Append(hv_Font + "-BoldItalic");
      hv_Guess.Append(hv_Font + "-BoldItalicMT");
      hv_Guess.Append(hv_Font + "-BoldOblique");
      {
        HTuple end_val127 = (hv_Guess.TupleLength()) - 1;
        HTuple step_val127 = 1;
        for (hv_I = 0; hv_I.Continue(end_val127, step_val127);
             hv_I += step_val127) {
          TupleFind(hv_SystemFonts, HTuple(hv_Guess[hv_I]), &hv_Index);
          if (0 != (hv_Index != -1)) {
            hv_Fonts[3] = HTuple(hv_Guess[hv_I]);
            break;
          }
        }
      }
    }
    hv_Font = ((const HTuple &)hv_Fonts)[hv_SubFamily];
    try {
      SetFont(hv_WindowHandle, (hv_Font + "-") + hv_Size);
    }
    // catch (Exception)
    catch (HalconCpp::HException &HDevExpDefaultException) {
      HDevExpDefaultException.ToHTuple(&hv_Exception);
      // throw (Exception)
    }
  } else {
    // Set font for UNIX systems
    hv_Size = hv_Size * 1.25;
    hv_AllowedFontSizes.Clear();
    hv_AllowedFontSizes[0] = 11;
    hv_AllowedFontSizes[1] = 14;
    hv_AllowedFontSizes[2] = 17;
    hv_AllowedFontSizes[3] = 20;
    hv_AllowedFontSizes[4] = 25;
    hv_AllowedFontSizes[5] = 34;
    if (0 != ((hv_AllowedFontSizes.TupleFind(hv_Size)) == -1)) {
      hv_Distances = (hv_AllowedFontSizes - hv_Size).TupleAbs();
      TupleSortIndex(hv_Distances, &hv_Indices);
      hv_Size = ((const HTuple &)hv_AllowedFontSizes)[HTuple(hv_Indices[0])];
    }
    if (0 != (HTuple(hv_Font == HTuple("mono"))
                  .TupleOr(hv_Font == HTuple("Courier")))) {
      hv_Font = "courier";
    } else if (0 != (hv_Font == HTuple("sans"))) {
      hv_Font = "helvetica";
    } else if (0 != (hv_Font == HTuple("serif"))) {
      hv_Font = "times";
    }
    if (0 != (hv_Bold == HTuple("true"))) {
      hv_Bold = "bold";
    } else if (0 != (hv_Bold == HTuple("false"))) {
      hv_Bold = "medium";
    } else {
      hv_Exception = "Wrong value of control parameter Bold";
      throw HalconCpp::HException(hv_Exception);
    }
    if (0 != (hv_Slant == HTuple("true"))) {
      if (0 != (hv_Font == HTuple("times"))) {
        hv_Slant = "i";
      } else {
        hv_Slant = "o";
      }
    } else if (0 != (hv_Slant == HTuple("false"))) {
      hv_Slant = "r";
    } else {
      hv_Exception = "Wrong value of control parameter Slant";
      throw HalconCpp::HException(hv_Exception);
    }
    try {
      SetFont(hv_WindowHandle,
              ((((((("-adobe-" + hv_Font) + "-") + hv_Bold) + "-") + hv_Slant) +
                "-normal-*-") +
               hv_Size) +
                  "-*-*-*-*-*-*-*");
    }
    // catch (Exception)
    catch (HalconCpp::HException &HDevExpDefaultException) {
      HDevExpDefaultException.ToHTuple(&hv_Exception);
      if (0 != (HTuple((hv_OS.TupleSubstr(0, 4)) == HTuple("Linux"))
                    .TupleAnd(hv_Font == HTuple("courier")))) {
        QueryFont(hv_WindowHandle, &hv_Fonts);
        hv_FontSelRegexp =
            (("^-[^-]*-[^-]*[Cc]ourier[^-]*-" + hv_Bold) + "-") + hv_Slant;
        hv_FontsCourier = (hv_Fonts.TupleRegexpSelect(hv_FontSelRegexp))
                              .TupleRegexpMatch(hv_FontSelRegexp);
        if (0 != ((hv_FontsCourier.TupleLength()) == 0)) {
          hv_Exception = "Wrong font name";
          // throw (Exception)
        } else {
          try {
            SetFont(hv_WindowHandle,
                    ((HTuple(hv_FontsCourier[0]) + "-normal-*-") + hv_Size) +
                        "-*-*-*-*-*-*-*");
          }
          // catch (Exception)
          catch (HalconCpp::HException &HDevExpDefaultException) {
            HDevExpDefaultException.ToHTuple(&hv_Exception);
            // throw (Exception)
          }
        }
      }
      // throw (Exception)
    }
  }
  // dev_set_preferences(...); only in hdevelop
  return;
}

void dev_open_window_fit_image(HObject ho_Image, HTuple hv_Row,
                               HTuple hv_Column, HTuple hv_WidthLimit,
                               HTuple hv_HeightLimit, HTuple *hv_WindowHandle) {
  // Local iconic variables

  // Local control variables
  HTuple hv_MinWidth, hv_MaxWidth, hv_MinHeight;
  HTuple hv_MaxHeight, hv_ResizeFactor, hv_ImageWidth, hv_ImageHeight;
  HTuple hv_TempWidth, hv_TempHeight, hv_WindowWidth, hv_WindowHeight;

  // This procedure opens a new graphics window and adjusts the size
  // such that it fits into the limits specified by WidthLimit
  // and HeightLimit, but also maintains the correct image aspect ratio.
  //
  // If it is impossible to match the minimum and maximum extent requirements
  // at the same time (f.e. if the image is very long but narrow),
  // the maximum value gets a higher priority,
  //
  // Parse input tuple WidthLimit
  if (0 !=
      (HTuple((hv_WidthLimit.TupleLength()) == 0).TupleOr(hv_WidthLimit < 0))) {
    hv_MinWidth = 500;
    hv_MaxWidth = 800;
  } else if (0 != ((hv_WidthLimit.TupleLength()) == 1)) {
    hv_MinWidth = 0;
    hv_MaxWidth = hv_WidthLimit;
  } else {
    hv_MinWidth = ((const HTuple &)hv_WidthLimit)[0];
    hv_MaxWidth = ((const HTuple &)hv_WidthLimit)[1];
  }
  // Parse input tuple HeightLimit
  if (0 != (HTuple((hv_HeightLimit.TupleLength()) == 0)
                .TupleOr(hv_HeightLimit < 0))) {
    hv_MinHeight = 400;
    hv_MaxHeight = 600;
  } else if (0 != ((hv_HeightLimit.TupleLength()) == 1)) {
    hv_MinHeight = 0;
    hv_MaxHeight = hv_HeightLimit;
  } else {
    hv_MinHeight = ((const HTuple &)hv_HeightLimit)[0];
    hv_MaxHeight = ((const HTuple &)hv_HeightLimit)[1];
  }
  //
  // Test, if window size has to be changed.
  hv_ResizeFactor = 1;
  GetImageSize(ho_Image, &hv_ImageWidth, &hv_ImageHeight);
  // First, expand window to the minimum extents (if necessary).
  if (0 != (HTuple(hv_MinWidth > hv_ImageWidth)
                .TupleOr(hv_MinHeight > hv_ImageHeight))) {
    hv_ResizeFactor =
        (((hv_MinWidth.TupleReal()) / hv_ImageWidth).TupleConcat(
             (hv_MinHeight.TupleReal()) / hv_ImageHeight)).TupleMax();
  }
  hv_TempWidth = hv_ImageWidth * hv_ResizeFactor;
  hv_TempHeight = hv_ImageHeight * hv_ResizeFactor;
  // Then, shrink window to maximum extents (if necessary).
  if (0 != (HTuple(hv_MaxWidth < hv_TempWidth)
                .TupleOr(hv_MaxHeight < hv_TempHeight))) {
    hv_ResizeFactor =
        hv_ResizeFactor *
        ((((hv_MaxWidth.TupleReal()) / hv_TempWidth).TupleConcat(
              (hv_MaxHeight.TupleReal()) / hv_TempHeight)).TupleMin());
  }
  hv_WindowWidth = hv_ImageWidth * hv_ResizeFactor;
  hv_WindowHeight = hv_ImageHeight * hv_ResizeFactor;
  // Resize window
  SetWindowAttr("background_color", "black");
  OpenWindow(hv_Row, hv_Column, hv_WindowWidth, hv_WindowHeight, 0, "", "",
             &(*hv_WindowHandle));
  HDevWindowStack::Push((*hv_WindowHandle));
  if (HDevWindowStack::IsOpen())
    SetPart(HDevWindowStack::GetActive(), 0, 0, hv_ImageHeight - 1,
            hv_ImageWidth - 1);
  return;
}
/*
void toImageMsg(HObject ho_image, sensor_msgs::Image &ros_image){
  HTuple hv_Width, hv_Height;
  GetImageSize(ho_image, &hv_Width, &hv_Height);
  ros_image.height = static_cast<int>(hv_Height);
  ros_image.width = static_cast<int>(hv_Width);
  ros_image.is_bigendian = 0;
  ros_image.step = ros_image.width * 8;
  size_t size = ros_image.step * ros_image.height;
  ros_image.data.resize(size);
  unsigned char *u_ptr;
  HTuple h_ptr;
  HTuple h_type;
  char typ[128];
  GetImagePointer1(ho_image, &h_ptr, &h_type, &hv_Width, &hv_Height);
  u_ptr = (unsigned char*)((Hlong) h_ptr);
  memcpy((char*)(&ros_image.data[0]), u_ptr, size);
}
*/

void toHImage(const sensor_msgs::Image::ConstPtr &ros_image, HImage *h_image) {
  int i;
  unsigned char *red, *green, *blue;
  red = (unsigned char *)malloc(sizeof(unsigned char) * ros_image->width *
                                ros_image->height);
  green = (unsigned char *)malloc(sizeof(unsigned char) * ros_image->width *
                                  ros_image->height);
  blue = (unsigned char *)malloc(sizeof(unsigned char) * ros_image->width *
                                 ros_image->height);
  for (i = 0; i < ros_image->width * ros_image->height; i++) {
    red[i] = ros_image->data[i * 3];
    green[i] = ros_image->data[i * 3 + 1];
    blue[i] = ros_image->data[i * 3 + 2];
  }

  h_image->GenImage3("byte", (int)ros_image->width, (int)ros_image->height, (void*)red,
                     (void*)green, (void*)blue);
  free(red);
  free(green);
  free(blue);
}
void toHObject(const sensor_msgs::Image::ConstPtr &ros_image,
               HObject *ho_image) {
  int i;
  unsigned char *red, *green, *blue;
  red = (unsigned char *)malloc(sizeof(unsigned char) * ros_image->width *
                                ros_image->height);
  green = (unsigned char *)malloc(sizeof(unsigned char) * ros_image->width *
                                  ros_image->height);
  blue = (unsigned char *)malloc(sizeof(unsigned char) * ros_image->width *
                                 ros_image->height);
  for (i = 0; i < ros_image->width * ros_image->height; i++) {
    red[i] = ros_image->data[i * 3];
    green[i] = ros_image->data[i * 3 + 1];
    blue[i] = ros_image->data[i * 3 + 2];
  }

  GenImage3(ho_image, "byte", (int)ros_image->width, (int)ros_image->height,
            (Hlong)red, (Hlong)green, (Hlong)blue);
  free(red);
  free(green);
  free(blue);
}

void toImageMsg(HObject ho_image, sensor_msgs::Image &ros_image) {
  HTuple hv_Width, hv_Height;
  HImage h_image(ho_image);

  GetImageSize(ho_image, &hv_Width, &hv_Height);
  //  ros_image.header = sensor_msgs::header();
  ros_image.height = static_cast<int>(hv_Height);
  ros_image.width = static_cast<int>(hv_Width);
  ros_image.encoding = "rgb8";
  ros_image.is_bigendian = 0;
  ros_image.step = ros_image.width * 3;
  size_t size = ros_image.step * ros_image.height;
  ros_image.data.resize(size);
  unsigned char *u_ptr;
  unsigned char *u_ptr_r;
  unsigned char *u_ptr_g;
  unsigned char *u_ptr_b;
  HTuple h_ptr;
  HTuple h_ptr_r;
  HTuple h_ptr_g;
  HTuple h_ptr_b;

  HTuple h_type;
  char typ[128];
  GetImagePointer1(ho_image, &h_ptr, &h_type, &hv_Width, &hv_Height);
  u_ptr = (unsigned char *)((Hlong)h_ptr);

  GetImagePointer3(ho_image, &h_ptr_r, &h_ptr_g, &h_ptr_b, &h_type, &hv_Width,
                   &hv_Height);
  u_ptr_r = (unsigned char *)((Hlong)h_ptr_r);
  u_ptr_g = (unsigned char *)((Hlong)h_ptr_g);
  u_ptr_b = (unsigned char *)((Hlong)h_ptr_b);

  // memcpy((char*)(&ros_image.data[0]), u_ptr, size);
  int i, k;

  for (i = 0; i < ros_image.width * ros_image.height; i++) {
    ros_image.data[i * 3] = u_ptr_r[i];
    ros_image.data[i * 3 + 1] = u_ptr_g[i];
    ros_image.data[i * 3 + 2] = u_ptr_b[i];
  }
}


// Procedures
// External procedures
// Chapter: Develop
// Short Description: Switch dev_update_pc, dev_update_var and dev_update_window
// to 'off'.
void dev_update_off() {
  // This procedure sets different update settings to 'off'.
  // This is useful to get the best performance and reduce overhead.
  //
  // dev_update_pc(...); only in hdevelop
  // dev_update_var(...); only in hdevelop
  // dev_update_window(...); only in hdevelop
  return;
}

};
