/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
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

_define_
Potentiometer (Process f, double _x, double _y, double _r) {
  Translation pos (_x, _y)
  Double output (0)
  Double angle (0)
  Double to_rad (3.14159/180)
  double g_r = _r + 1

  Int bkg_color (#808080)
  Int fg_color (#D6D6D6)
  Int needle_color (#2B2B2B)
  Int active_bkg_color (#00ff67)

  /* graphics */
  Component rotary_button {
    FillColor _bkg_color (#808080)
    bkg_color =:> _bkg_color.value
    Circle _ (0,0,_r)
    FillColor _active_bkg_color (#00ff67)
    active_bkg_color =:> _active_bkg_color.value

    SwitchRange range (10) {
      br_0_90 [10, 180[ {
        Path p {
          PathMove m(0, g_r)
          PathArc a (g_r, g_r, 0, 0, 1, -_r,0)
          PathLine _ (0, 0)
          PathClosure _
        }
        cos (-260 * to_rad) * g_r =: p.m.x
        sin (-260 * to_rad) * g_r =: p.m.y
        cos ((angle - 270) * to_rad) * g_r =:> p.a.x
        sin ((angle - 270) * to_rad) * g_r =:> p.a.y
      }
      br_180_270 [180, 340] {
        Path p {
          PathMove m (0, g_r)
          PathArc a (g_r, g_r, 0, 0, 1, 0, -g_r)
          PathLine _ (0, 0)
          PathClosure _
        }
        Path p2 {
          PathMove _(0, -g_r)
          PathArc a (g_r, g_r, 0, 0, 1, 0, 0)
          PathLine _ (0, 0)
          PathClosure _
        }
        cos (-260 * to_rad) * g_r =: p.m.x
        sin (-260 * to_rad) * g_r =: p.m.y
        cos ((angle + 90) * to_rad) * g_r =:> p2.a.x
        sin ((angle + 90) * to_rad) * g_r =:> p2.a.y
      }
    }
    Rotation rot (10, 0, 0)
    rot.a =:> angle, range.input
    NoOutline _
    FillColor _fg_color (#D6D6D6)
    fg_color =:> _fg_color.value
    Circle button (0,0, _r - 5)
    OutlineColor _needle_color (#2B2B2B)
    needle_color =:> _needle_color.value
    OutlineWidth _ (3)
    Line needle (0, 0, 0, _r - 5)
  }
  
  /* Behavior */
  BoundedValue bd (10, 340, 10)
  ((rotary_button.rot.a - 10) / 330) =:> output
  Double debug (0)
  Double cur_a (0)
  Double to_deg (180/3.14159)
  Component p1 {
    Double x (_x)
    Double y (_y)
  }
  Component p2 {
    Double x (0)
    Double y (0)
  }
  Component p3{
    Double x (0)
    Double y (0)
  }
  Double buff (0)
  Double new_angle (0)

  Bool min (0)
  Bool max (0)
  rotary_button.rot.a == 340 =:> max
  rotary_button.rot.a == 10 =:> min
  Spike move
  FSM fsm {
    State idle {
      rotary_button.rot.a =: cur_a
    }
    State moving {
      f.press.x =: p2.x
      f.press.y =: p2.y
      rotary_button.button.move.x =:> p3.x
      rotary_button.button.move.y =:> p3.y
      (atan2(p3.y - p1.y, p3.x - p1.x) - atan2(p2.y - p1.y, p2.x - p1.x)) =:> new_angle
      cur_a + (new_angle * to_deg) =:> buff
      buff < 0 ? fmod (360 + buff, 360) : fmod ($buff, 360) =:> bd.input
      bd.result =:> rotary_button.rot.a
    }
    State st_min
    {
      rotary_button.button.move.x =:> p3.x
      rotary_button.button.move.y =:> p3.y
      (atan2(p3.y - p1.y, p3.x - p1.x) - atan2(p2.y - p1.y, p2.x - p1.x)) =:> new_angle
      cur_a + (new_angle * to_deg) =:> buff
      buff < 0 ? fmod (360 + buff, 360) : fmod ($buff, 360) =:> bd.input
      (bd.result > 10 && bd.result < 50)->move
    }
    State st_max
    {
      rotary_button.button.move.x =:> p3.x
      rotary_button.button.move.y =:> p3.y
      (atan2(p3.y - p1.y, p3.x - p1.x) - atan2(p2.y - p1.y, p2.x - p1.x)) =:> new_angle
      cur_a + (new_angle * to_deg) =:> buff
      buff < 0 ? fmod (360 + buff, 360) : fmod ($buff, 360) =:> bd.input
      (bd.result < 340 && bd.result >290)->move     
    }
    idle->moving (rotary_button.button.press)
    {moving, st_max, st_min}->idle (f.release)
    moving->st_max (max.true)
    moving->st_min (min.true)
    st_min->moving (move)
    st_max->moving (move)
  }
}