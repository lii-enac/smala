/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
use core
use base
use display
use gui

_native_code_
%{
#include <cmath>
%}


_action_
smala_build_pie_menu (Process src, Process data)
{     
  double PI = 3.14159

  items_list = getRef (data.model)

  double number_slice = data.model_size.value
  double offset_angle = (number_slice <= 2) ? 0 : -90 // start at noon except for 1 and 2 slices
  double outer_radius = $data.outer_radius
  double inner_radius = $data.inner_radius
  double slice_angle = 360.0 / number_slice

  for (int i = 0; i < number_slice; i++) {
    double start_angle = offset_angle + i * slice_angle
    double end_angle = start_angle + slice_angle

    double theta0 = start_angle * PI / 180
    double theta1 = end_angle * PI / 180

    double x0 = outer_radius * cos(theta0)
    double y0 = outer_radius * sin(theta0)

    double x1 = outer_radius * cos(theta1)
    double y1 = outer_radius * sin(theta1)

    double x2 = inner_radius * cos(theta1)
    double y2 = inner_radius * sin(theta1)

    double x3 = inner_radius * cos(theta0)
    double y3 = inner_radius * sin(theta0)

    //center for text
    double mid_angle = (start_angle + end_angle) / 2.0
    double mid_radius = (inner_radius + outer_radius) / 2.0

    double theta = (number_slice > 1 ) ? mid_angle * PI / 180 : -(mid_angle / 2) * PI / 180
    double cx = mid_radius * cos(theta)
    double cy = mid_radius * sin(theta)

    double text_angle = (number_slice > 1 ) ? mid_angle + 90 : mid_angle
  
    if (number_slice < 2 || cy > 0) {
      text_angle += 180
    }

    addChildrenTo data.content {
      Component _ {
        // background slice
        Path _ {
          PathMove _ (x0, y0)
          PathArc _ (outer_radius, outer_radius, 0, (slice_angle > 180 ? 1 : 0), 1, x1, y1)

          if (number_slice > 1) {
            PathLine _ (x2, y2)
            PathArc _ (inner_radius, inner_radius, 0, (slice_angle > 180 ? 1 : 0), 0, x3, y3)
            PathLine _ (x0, y0)
          } else {
            PathMove _ (x2, y2)
            PathArc _ (inner_radius, inner_radius, 0, 1, 0, x3, y3)
          }

          PathClosure _
        } 

        // label slice
        Component label {
          Translation t (cx, cy)
          Rotation _ (text_angle, 0, 0)
          FillColor fc ($data.foreground_color)
          data.foreground_color =:> fc.value
          FontSize fs (5, $data.text_size_px) // 5 = pixel
          data.text_size_px =:> fs.size
          Translation t2 (0, 0)
          TextAnchor _ (1) // middle
          Text text (0, 0, getString (items_list.[i+1]))
          if (cy > 0 ) {
            - $text.height / 2 =: t2.ty
          }
          else {
            $text.height / 2 =: t2.ty
          }
        }

        // mask slice 
        FillColor fcm ($data.mask_color)
        data.mask_color =:> fcm.value
        FillOpacity op (0)
        Path mask {
          PathMove _ (x0, y0)
          PathArc _ (outer_radius, outer_radius, 0, (slice_angle > 180 ? 1 : 0), 1, x1, y1)

          if (number_slice > 1) {
            PathLine _ (x2, y2)
            PathArc _ (inner_radius, inner_radius, 0, (slice_angle > 180 ? 1 : 0), 0, x3, y3)
            PathLine _ (x0, y0)
          } else {
            PathMove _ (x2, y2)
            PathArc _ (inner_radius, inner_radius, 0, 1, 0, x3, y3)
          }

          PathClosure _
        }

        FSM fsm_mask {
          State idle {
            0 =: op.a
          }
          State hover {
            0.5 =: op.a
          }
          idle -> hover (mask.enter)
          hover -> idle (mask.leave)
        }

        // connector to selected value and close pie menu when selection
        mask.release -> {
          items_list.[i+1] =: data.value
          "false" =: data.is_open
        } 
      }
    }

  
  }
}

_define_
PieMenu (Process _frame, double x_, double y_, int _inner_radius, int _outer_radius) {
  
  mouseTracking = 1
  Translation t (x_, y_)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  frame aka _frame
  RefProperty model (0)
  DerefInt model_size (model, "size", DJNN_GET_ON_CHANGE)
  Int outer_radius (_outer_radius)
  Int inner_radius (_inner_radius)
  String value ("")
 
  Int background_color (#000000)
  Int foreground_color (#FFFFFF)
  Int mask_color (#FFFFFF)
  Int outline_width (3)
  Int text_size_px (15)

  Bool is_open (false)

  /*----- interface -----*/

  NativeAction na_build_pie_menu (smala_build_pie_menu, this, 1)
  model_size.value -> na_build_pie_menu

  Switch sw_open_close (false) {
    Component true {
      Component content {
        FillColor bgc ($background_color)
        background_color =:> bgc.value
        OutlineColor olc ($foreground_color)
        foreground_color =:> olc.value
        OutlineWidth ow ($outline_width)
        outline_width =:> ow.width
      }
    }
    Component false
  }
  is_open =:> sw_open_close.state

  content aka sw_open_close.true.content

}
