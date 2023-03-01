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
*    Stephane Conversy <stephane.conversy@enac.fr>
*
*/

use core
use base
use display
use gui

import Scrollbar

_main_
Component root {
  Frame f ("MDPC scrollbar", 0, 0, 800, 800)
  Exit ex (0, 1)
  f.close -> ex

  Scrollbar ysb(f)

  // the default scrollbar is vertical, pointing upward, x-centered
  // we need to make ysb pointing downward since this is typical window coordinates
  // so rotate it by half a circle
  180 =:> ysb.transform.rot
  // put it at the left of the frame
  f.width - ysb.width/2 =:> ysb.transform.tx
  // since it's rotated 180, we need to translate it by its size // FIXME: y-centered?
  f.height + 0 =:> ysb.transform.ty
  // and make as high as the frame
  f.height - ysb.arrow_height*2 =:> ysb.transform.s
  // arbitrary settings, not related to the scrollbar model
  //50 =: ysb.width
  //50 =: ysb.arrow_height
  // debug
  //f.press.x =:> ysb.transform.rot
  //-20 =:> ysb.pick_xoffset

  Scrollbar xsb(f)

  // the default scrollbar is vertical, pointing upward
  // make it horizontal by rotating it by 90°
  90 =:> xsb.transform.rot
  // put it at the bottom of the frame
  f.width - ysb.arrow_height =:> xsb.transform.tx
  f.height - xsb.width/2 =:> xsb.transform.ty
  // and make as large as the frame, but do not overlap ysb arrow
  f.width - xsb.arrow_height*2 - ysb.width  =:> xsb.transform.s
  // arbitrary settings, not related to the model
  //50 =: xsb.width
  //50 =: xsb.arrow_height
  // debug
  //f.press.x =:> xsb.transform.rot
  //-20 =:> xsb.pick_xoffset

  // the scrollbar model is two values between 0 and 1
  // let's say that the view is 1600x1600, so the initial values are
  0 =: ysb.model.low
  f.height / 1600 =: ysb.model.high
  0 =: xsb.model.low
  f.width / 1600 =: xsb.model.high
  
  // not yet: // and whenever the user resizes the frame, the scrollbar is updated through model updates

  // graphical scene that scrolls accorging to the scrollbars state
  Component scene {
    Translation tr(0,0)
    //Scaling sc(1,1, 0,0)

    -xsb.model.low * 1600 =:> tr.tx
    -ysb.model.low * 1600 =:> tr.ty
    //sb.model.delta * 1600 =:> sc.sy
    // TODO: make an animation when it's a single paging

    FillColor _ (255,0,0)
    Circle _(50,50,50)

    FillColor _ (255, 255,0)
    Circle _(50,950,50)

    FillColor _ (0, 255,0)
    Circle _(950,950,50)

    FillColor _ (0, 255, 255)
    Circle _(950,50,50)
  }

  // debugging scrollbar model state
  // FillColor _(255,255,255)
  // Text ylow(0,10,"")
  // Text yhigh(0,25,"")
  // toString(ysb.model.low) =:> ylow.text
  // toString(ysb.model.high) =:> yhigh.text

}
