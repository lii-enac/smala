use core
use base
use gui


/*
 * Computes pan and mouse centered zoom of a component transformed by a scaling then a translation.
 * Works by connecting interface members to these scaling and translation.
 * Works whatever the transforms applied before the pan and zoom branch of the components tree.
 * If used in combination with a drag interaction, necessitates a rectangle background passed as 
 * bg argument (or else drag will conflict with pan).
 * If no drag, pass the frame both as frame and bg parameters.
 */
_define_
PanAndZoom (Component frame, Component bg) {

  /*----- interface -----*/
  Double zoom (1)
  Double xpan (0)
  Double ypan (0)
  /*--- end interface ---*/

  // Non visual rectangle serving as a local coordinates system
  Rectangle localRef (0, 0, 0, 0, 0, 0)

  // Pan management
  FSM fsm {
    State idle

    State pressed {
      // Memorize press position in local coordinates
      ScreenToLocal s2l (localRef)
      frame.press.x =: s2l.inX
      frame.press.y =: s2l.inY
    }

    State panning {
      // Memorize initial translation
      AdderAccumulator atx (0, 0, 0)
      xpan :: atx.input
      AdderAccumulator aty (0, 0, 0)
      ypan :: aty.input

      // Convert mouse move to local coordinates
      ScreenToLocal s2l (localRef)
      frame.move.x => s2l.inX
      frame.move.y => s2l.inY

      // Compute added translation
      s2l.outX - pressed.s2l.outX => atx.input
      s2l.outY - pressed.s2l.outY => aty.input
      atx.result => xpan
      aty.result => ypan
    }

    State zooming {
      // init a clock to leave this state after a timeout without interaction
      Clock cl (1000)
      
      // compute scale factor to modify zoom (data flow)
      Pow p (1.01, 0)
      Double scaleFactor (1)
      frame.wheel.dy => p.exponent
      p.result => scaleFactor

      // store pointer position in localRef coord system (data flow)
      Double p0x (0)
      Double p0y (0)
      ScreenToLocal s2l (localRef)
      frame.move.x => s2l.inX
      frame.move.y => s2l.inY
      s2l.outX => p0x
      s2l.outY => p0y

      // changing zoom then translating (sequence activated on scaleFactor activation)
      AssignmentSequence setZoomAndPan (0) {
        // apply new zoom
        zoom * scaleFactor =: zoom
        // After the zoom is applied, p0 has become p1 = p0 * scaleFactor
        // So to make believe that zoom is mouse centered, we must apply
        // a new translation (p0 - p1) taking the scaleFactor into account
        // i.e replace pan by (pan + p0 - p1)/scaleFactor
        (xpan + p0x * (1 - scaleFactor)) / scaleFactor =: xpan
        (ypan + p0y * (1 - scaleFactor)) / scaleFactor =: ypan
      }
      scaleFactor -> setZoomAndPan, cl
    }

    idle -> pressed (bg.press)
    pressed -> idle (frame.release)
    pressed -> panning (frame.move)
    panning -> idle (frame.release)
    idle -> zooming (frame.wheel)
    zooming -> idle (zooming.cl.tick)
    zooming -> pressed (bg.press)
  }

}