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
*    Nicolas Saporito  <nicolas.saporito@enac.fr>
*    Stephane Conversy <stephane.conversy@enac.fr>
*
*/

use core
use base
use display
use gui

import Drag
import PanAndZoom

_main_
Component root {

  Frame frame ("Drag, Pan, Zoom", 0, 0, 800, 800)
  Exit ex (0, 1)
  frame.close -> ex
  mouseTracking = 1

  // background to track mouse presses outside draggable shapes in order to prevent pan/drag conflict
  // (listening frame.press doesn't allow to differentiate press inside or outside shapes)
  Component backgroundForPan {
    NoOutline _
    NoFill _
    Rectangle bg (0, 0, 0, 0, 0, 0)
    frame.width =:> bg.width
    frame.height =:> bg.height
  }

  // A few arbitrary transforms
  Rotation arbitraryRotation (19, 0, 0)
  Translation arbitraryTranslation (56, 24)
  Scaling arbitraryScaling (1.2, 1.2, 0, 0)

  Component zoomScene {
    // Pan and zoom
    Homography panAndZoomTransform
    PanAndZoom pz (frame, backgroundForPan.bg, panAndZoomTransform)

    FillColor _ (70, 70, 70)
    Text ts (200, 200, "The scene can be panned and zoomed, the rectangle can be dragged")

    Component draggableRect {
      // Homography to accumulate drags
      Homography dragTransform
      // styles
      FillColor _ (200, 200, 200)
      FillOpacity _ (0.5)
      OutlineColor _ (70, 70, 70)
      // adaptive outline width (always 1px)
      OutlineWidth ow (1)
      1 / panAndZoomTransform.accsx =:> ow.width
      // graphics
      Rectangle rectangle (100, 100, 100, 100, 0, 0)
      // drag
      Drag drag (frame, rectangle, dragTransform)
    }

    Component zoomCoordOnly {
      // position
      Translation t (100, 300)
      // apply inverse of the accumulated zoom (supposedly homothetic)
      Scaling scaling_inv (1, 1, 0, 0)
      1 / panAndZoomTransform.accsx =:> scaling_inv.sx, scaling_inv.sy
      // graphics
      FillColor _ (125, 125, 125)
      Text te (0, 0, "...except this text wich moves as a consequence of pan and zoom, but does not scale")
    }
  }

  Component stickToScreenScene {
    // This component is not concerned at all by the transforms in the zoomScene component
    FillColor _ (0, 0, 0)
    Text t (100, 400, "... and this text which is not transformed at all")
  }
}

run root
run syshook
