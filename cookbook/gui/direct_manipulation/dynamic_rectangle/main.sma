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
*    
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

_action_
delete_rectangle (Process src, Process data)
{
  grect = find (&src, "../../..")
  delete grect
}

_main_
Component root {
  
  Frame frame ("dynamic rectangle", 0, 0, 600, 600)
  //Exit ex (0, 1)
  frame.close ->! mainloop


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

  frame.
  background_rect.
  press -> (root) {
    addChildrenTo root.things {
      /* we have to create a group to delete both the rect and the associated binding */
      Component grect {
        //Rectangle rect ( $root.frame.background_rect.press.x, $root.frame.background_rect.press.y, 100, 100, 0, 0)
        Translation _($root.frame.background_rect.press.x, $root.frame.background_rect.press.y)
        
        Component _ {
          Translation _(-750,-250)
          Polygon rect {
            Point _(867.,          256.        )
            Point _(945.98376465,  385.31274414)
            Point _(798.5927124,   350.15460205)
            Point _(700.01623535,  465.23242188)
            Point _(687.9072876,   314.19073486)
            Point _(548.,          256.        )
            Point _(687.9072876,   197.80926514)
            Point _(700.01623535,   46.76756668)
            Point _(798.5927124,   161.84539795)
            Point _(945.98376465,  126.68724823)
          }
          rect.press -> root.na_delete_rectangle
         
        }
        //FillColor _(#FFFFFF)
        //Text _(0,0, "yop")
        //Rectangle rect (-50, -50, 100, 100, 0, 0)
        //rect.press -> root.na_delete_rectangle
      }
    }
  }

  FillColor _ (#FF0000)

  //Text explanation1 (10, 20, "Press and release on the window to create a rectangle")
  //Text explanation2 (10, 40, "then press and release a rectangle to delete it !")

  
}





