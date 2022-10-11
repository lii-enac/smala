use core
use base
use display
use gui

import Module

_native_code_
%{
#include "user_defined_struct.h"
MyPoint2D pdef{0,0};
vector_double vd_def;
%}

_main_
Component root
{
  Frame f ("my frame", 0, 0, 500, 300)
  Exit ex (0, 1)
  f.close -> ex

  TextPrinter tp

  // user-defined type
  Module mod()
  TemplateProperty of MyPoint2D p0(pdef)
  TemplateProperty of MyPoint2D p1 aka mod.p1 // cast an external property into a TemplateProperty
  TemplateProperty of MyPoint2D p2(pdef)
  p0 + p1 =:> p2
  //p2 =:> tp.input

  // units
  TemplateProperty of flightLevel_t Altitude(0_ft)
  TemplateProperty of speed_t speed(0_mps)
  TemplateProperty of meter_t m(14_m)
  TemplateProperty of second_t s(2_s)
  m / s =:> speed
  to_string2($speed) =:> tp.input
  
  // array of double
  TemplateProperty of vector_double dba(vd_def)
  TemplateProperty of vector_double dbb(vd_def)
  TemplateProperty of vector_double dbc(vd_def)
  dba + dbb =:> dbc

}

