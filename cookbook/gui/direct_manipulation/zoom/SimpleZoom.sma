use core
use base
use gui

/*
 * Computes pan and mouse centered zoom of a component transformed by a scaling then a translation.
 * Works by connecting interface members to these scaling and translation.
 */
_define_
SimpleZoom (Component frame, Component scaling) {

  Double zoom (1)
  zoom =:> scaling.sx, scaling.sy
  frame.width /2 =:> scaling.cx
  frame.height /2 =:> scaling.cy

  Pow p (1.01, 0)
  frame.wheel.dy =:> p.exponent

  AssignmentSequence seq (0) {
    // apply new zoom
    zoom * p.result =: zoom
  }
  p.result -> seq

}