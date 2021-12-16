/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020-2021)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */
use core
use base
use display
use gui

_native_code_
%{
  #include <iostream>
  using namespace std;
%}


_define_
RemoteCursor (string label_p, double x_p, double y_p, int color_p)
{
  /*----- CONST -----*/
  Double MOVING_OPACITY (0.75)
  Double PAUSED_OPACITY (0.25)
  Double STOPPED_OPACITY (0.0)
  /*----- CONST -----*/

  Translation t (x_p, y_p)

  Spike move
  Spike pause

  FillOpacity fill_opacity (0.8)
  OutlineOpacity outline_opacity (0.8)

  OutlineWidth _(1)
  OutlineColor _(255, 255, 255)
  FillColor fill_c (color_p)
  
  Polygon arrow {
    Point _(0, 0)
    Point _(0, 14)
    Point _(3, 11)
    Point _(5, 17)
    Point _(8, 16)
    Point _(6, 10)
    Point _(10, 10)
  }

  FillColor _ (0, 0, 0)
  FontSize _ (0, 10)
  Text label (12, 12, label_p)
  

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  color aka fill_c.value
  uid aka label.text
  /*----- interface -----*/


  x -> move
  //y -> move

  FSM fsm {
    State moving {
      MOVING_OPACITY =: fill_opacity.a, outline_opacity.a
    }
    State short_pause {
      Timer t (1000)
    }
    State long_pause {
      PAUSED_OPACITY =: fill_opacity.a, outline_opacity.a
      Timer t (5000)
    }
    State stopped {
      STOPPED_OPACITY =: fill_opacity.a, outline_opacity.a
    }
    moving -> short_pause (pause)
    short_pause -> moving (move)
    short_pause -> long_pause (short_pause.t.end)
    long_pause -> moving (move)
    long_pause -> stopped (long_pause.t.end)
    stopped -> moving (move)
  }
}