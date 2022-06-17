/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.ComboBox
import gui.widgets.VBox
_main_
Component root {
  Frame f ("simplest", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  VBox vbox (f)
  vbox.space = 20
  PushButton pb ("Button1")
  ComboBox cb
  addChildrenTo cb.str_items {
    String _ ("first item")
    String _ ("long second item")
    String _ ("third item")
  }
  cb.preferred_width = 100
  PushButton pb2 ("Button1")
  addChildrenTo vbox.items {
    pb, cb, pb2
  }
}
