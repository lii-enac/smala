use core
use exec_env
use base
use display
use gui

/*
 * Computes mouse-centered zoom of a component transformed by a scaling then a translation.
 * Works by connecting interface members to these scaling and translation.
 */
_define_
MouseCenteredZoom (Process frame, Process scaling, Process translation) {

  Double zoom (1)
  zoom =:> scaling.sx, scaling.sy
  Double xpan (0)
  xpan =:> translation.tx
  Double ypan (0)
  ypan =:> translation.ty

  Pow p (1.01, 0)
  Double scaleFactor (1)
  frame.wheel.dy =:> p.exponent
  p.result =:> scaleFactor

  Double p0x (0) // mouse x in local coord system before zoom
  Double p0y (0)

  AssignmentSequence seq (0) {
    // remember mouse pos in local coord system before zoom
    // (as a local coord it won't change after zoom is applied)
    frame.move.x / zoom - xpan =: p0x
    frame.move.y / zoom - ypan =: p0y
    // apply new zoom
    zoom * scaleFactor =: zoom
    // After the zoom is applied, p0 has become p1 = p0 * scaleFactor
    // So to make believe that zoom is mouse centered, we must apply
    // a new translation (p0 - p1) taking the scaleFactor into account
    // i.e replace pan by (pan + p0 - p1)/scaleFactor
    (xpan + p0x * (1 - scaleFactor)) / scaleFactor =: xpan
    (ypan + p0y * (1 - scaleFactor)) / scaleFactor =: ypan
  }
  scaleFactor -> seq

}