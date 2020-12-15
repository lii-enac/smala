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
 *    Nicolas Saporito <nicolas.saporito@enac.fr>
 *
 */

use core
use base
use display
use gui


/*
 * Computes pan and mouse centered zoom.
 * All the transforms are accumulated into the Homography passed as transforms argument.
 * Works whatever the transforms applied before the pan and zoom branch of the components tree
 * (right matrix product in the homography, so in the local coordinate system of the node after homography).
 * If used in combination with a drag interaction, necessitates a rectangle background passed as 
 * bg argument (or else drag will conflict with pan).
 * If no drag, pass the frame both as frame and bg parameters.
 */
_define_
PanAndZoom (Process frame, Process bg, Process transforms) {

  // Non visual rectangle serving as a local coordinates system
  Rectangle localRef (0, 0, 0, 0, 0, 0)

  // Store pointer position in localRef coordinates system
  // (used by both zoom and pan management)
  ScreenToLocal s2l (&localRef)

  // Zoom management
  // set scaling center according to zoom option
  Switch mouseCenteredOption (off) {
    // frame centered zoom option
    Component off {
      // set scaling center to the center of the frame (localRef coordinates system)
      ScreenToLocal s2l (&localRef)
      AssignmentSequence seq (1) {
        frame.width /2  =: s2l.inX
        frame.height /2 =: s2l.inY
      }
      // the coordinates conversion must be activated by frame wheel because matrix 
      // could have changed due to other transforms whereas frame dimensions rarely change,
      // so simple connectors (like in the mouse-centered option below) won't do the job
      frame.wheel -> seq
      // no need to propagate the scaling center change for the moment,
      // as the sx/sy scaling factors haven't been computed yet
      s2l.outX =:> transforms.rightScaleBy.cx
      s2l.outY =:> transforms.rightScaleBy.cy
    }

    // mouse centered zoom option
    Component on {
      // set scaling center to pointer position (localRef coordinates system)
      ScreenToLocal s2l (&localRef)
      frame.move.x =:> s2l.inX
      frame.move.y =:> s2l.inY
      s2l.outX =:> transforms.rightScaleBy.cx
      s2l.outY =:> transforms.rightScaleBy.cy
    }
  }

  Pow p (1.01, 0)
  frame.wheel.dy =:> p.exponent
  p.result =:> transforms.rightScaleBy.sx, transforms.rightScaleBy.sy

  // Pan management
  FSM panFsm {
    State idle

    State pressed {
      // Memorize press position in local coordinates
      frame.press.x =: s2l.inX
      frame.press.y =: s2l.inY
    }

    State panning {
      Double init_x (0)
      Double init_y (0)
      s2l.outX =: init_x
      s2l.outY =: init_y
      frame.move.x =:> s2l.inX
      frame.move.y =:> s2l.inY
      // Compute added translation
      s2l.outX - init_x =:> transforms.rightTranslateBy.dx
      s2l.outY - init_y =:> transforms.rightTranslateBy.dy
    }

    idle -> pressed (bg.press)
    pressed -> idle (frame.release)
    pressed -> panning (frame.move)
    panning -> idle (frame.release)
  }
}