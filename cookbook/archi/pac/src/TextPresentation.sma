/*
*  Smala cookbook pac
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*
*/
use core
use base
use gui

_define_
TextPresentation(Process abstraction, int _ty) {
  // TextPresentations is a textual view of a conceptual rectangle...
  // It displays the rect 4 properties as 4 texts (for x,y,w and h) on the window

  Component view {
    Translation pos (0, _ty)

    // the 4 text graphical objects...
    Text t_x (0, 15, "0")
    Text t_y (15, 15, "0")
    Text t_width (30, 15, "0")
    Text t_height (45, 15, "0")

    // ... laid out horizontally
    t_x.x     + t_x.width     + 5 =:> t_y.x
    t_y.x     + t_y.width     + 5 =:> t_width.x
    t_width.x + t_width.width + 5 =:> t_height.x

    // the TextView 'api' changes the textual content of the 4 graphical texts
    x aka t_x.text
    y aka t_y.text
    width aka t_width.text
    height aka t_height.text
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