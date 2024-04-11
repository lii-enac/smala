/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*/

use core
use base
use gui

_define_
TextPresentation(Process abstraction, int _ty) {

  Component view {
    Translation pos (0, _ty)
    Text t_x (0, 15, "0")
    x aka t_x.text
    Text t_y (15, 15, "0")
    y aka t_y.text
    Text t_width (30, 15, "0")
    width aka t_width.text
    Text t_height (45, 15, "0")
    height aka t_height.text
    t_x.x + t_x.width + 5 =:> t_y.x
    t_y.x + t_y.width + 5 =:> t_width.x
    t_width.x + t_width.width + 5 =:> t_height.x
  }

  Component control {
    // update the view whenever the abstraction changes (subject/observer pattern)
    abstraction.{x,y,width,height} =:> view.{x,y,width,height}

    // update abstraction from interactions on the view
         view.t_x.wheel.dy +=> abstraction.x
         view.t_y.wheel.dy +=> abstraction.y
     view.t_width.wheel.dy +=> abstraction.width
    view.t_height.wheel.dy +=> abstraction.height
  }

}