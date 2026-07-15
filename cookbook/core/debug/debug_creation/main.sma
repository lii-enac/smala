/*
*  Smala cookbook debug_creation
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui
use utils

/* 

IMPORTANT NOTE ! 
Make sure djnn-cpp is compiled with the _DEBUG_SEE_CREATION_DESTRUCTION_ORDER option defined in process.h.

*/


_native_code_
%{
#include "utils/utils.h"
%}

_action_
delete_rectangle (Process src, Process data)
{
  grect = find (&src, "../..")
  delete grect
}

_main_
Component root {
  
  Frame frame ("dynamic rectangle", 0, 0, 600, 600)
  frame.close ->! mainloop

  FillColor _ (Orange)
  Rectangle rect_dump (10, 10, 10, 10, 0, 0)

  rect_dump.press -> (root) {
    display_creation_stats ()
  }


  FillColor fg_color (#373700)
  OutlineColor fg_color_outline (#FF0000)

  NativeAction na_delete_rectangle (delete_rectangle, root, 1)

  Translation lt(0,0)
  Layer things {
  //Component things {

  }

  frame.key\-pressed == DJN_Key_Right -> { //
    lt.tx + 10 =: lt.tx
  }

  frame.background_rect.press -> (root) {
    addChildrenTo root.things {
      /* we have to create a group to delete both the rect and the associated binding */
      Component grect {
        Rectangle rect ( $root.frame.background_rect.press.x, $root.frame.background_rect.press.y, 100, 100, 0, 0)
        
          rect.press -> root.na_delete_rectangle
         
        }
      }
    }
    
    FillColor _ (White)
    Text explanation1 (30, 20, "Press and release on the window to create a rectangle")
    Text explanation2 (30, 40, "then press and release a rectangle to delete it !")
    Text explanation3 (30, 60, "Also click the orange rectangle to dump a snapshot ")
    Text explanation4 (30, 80, "of the creation/destruction stats for the app.")
  }