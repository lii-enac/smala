/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2022)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*
*/
use core

_native_code_
%{
#include "user_defined_struct.h"
%}

_define_
Module () {
    TemplateProperty of MyPoint2D p1(pdef)
}

