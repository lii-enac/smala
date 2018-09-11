use core
use base
use gui


import ScreenToLocal


/*
 * Computes pan and mouse centered zoom of a component transformed by a scaling then a translation.
 * Works by connecting interface members to these scaling and translation.
 * Works whatever the transforms applied before the pan and zoom branch of the components tree.
 * If used in combination with a drag interaction, necessitates a rectangle background passed as 
 * bg argument (or else drag will conflict with pan).
 * If no drag, pass frame both as f and bg.
 */
_define_
PanAndZoom (Component f, Component bg) {

  /*----- interface -----*/
  Double zoom (1)
  Double xpan (0)
  Double ypan (0)
  /*--- end interface ---*/

  // Non visual rectangle serving as a local coordinates system
  Rectangle localRefRect (0, 0, 0, 0, 0, 0)

  Double pressX (0)
  Double pressY (0)

  // Pan management
  FSM fsm {
    State idle

    // Only in hysteresis version
    // With the hysteresis version, it's better to have a scene component (graphical elements 
    // along with their transforms) and a fsm in distinct branches of the components tree 
    // in order to keep the hysteresis circle drawing simple in the screen coordinates system
    // (with none of the transforms applied to the scene component)
    State waiting_hyst {
      // Memorize press position in local coordinates
      ScreenToLocal scr2loc (localRefRect)
      localRefRect.press.x =: scr2loc.screenX
      localRefRect.press.y =: scr2loc.screenY
      scr2loc.localX => pressX
      scr2loc.localY => pressY

      FillColor fcwh (255, 0, 0)
      Circle c (0, 0, 20)
      pressX => c.cx
      pressY => c.cy
    }

    State pressed {
      // Memorize press position in local coordinates
      ScreenToLocal scr2loc (localRefRect)
      f.press.x =: scr2loc.screenX
      f.press.y =: scr2loc.screenY
      scr2loc.localX => pressX
      scr2loc.localY => pressY
      // pressX/Y must be connected and not assigned as there is no guarantee that data flow
      // inside ScreenToLocal will be activated before the assignment on pressX/Y is done.
      // Using an AssignmentSequence here won't change anything.
    }

    State panning {
      // Memorize initial translation
      Double tx0 (0)
      Double ty0 (0)
      xpan =: tx0
      ypan =: ty0

      // Compute added translation
      ScreenToLocal scr2loc (localRefRect)
      f.move.x => scr2loc.screenX
      f.move.y => scr2loc.screenY
      tx0 + scr2loc.localX - pressX => xpan
      ty0 + scr2loc.localY - pressY => ypan
    }

    State zooming {
      // init a clock to leave this state after a timeout without interaction
      Clock cl (1000)
      
      // compute scale factor to modify zoom (data flow)
      Pow p (1.01, 0)
      Double scaleFactor (1)
      f.wheel.dy => p.exponent
      p.result => scaleFactor

      // store pointer position in localRefRect coord system (data flow)
      Double p0x (0)
      Double p0y (0)
      ScreenToLocal scr2loc (localRefRect)
      f.move.x => scr2loc.screenX
      f.move.y => scr2loc.screenY
      scr2loc.localX => p0x
      scr2loc.localY => p0y

      // changing zoom then translating (sequence activated on mouse wheel action)
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

    // Transitions for basic version
    idle -> pressed (bg.press)
    pressed -> idle (f.release)
    pressed -> panning (f.move)
    panning -> idle (f.release)
    idle -> zooming (f.wheel)
    zooming -> idle (zooming.cl.tick)
    zooming -> pressed (bg.press)
  
    /*
    // Transitions for hysteresis version
    idle -> waiting_hyst (bg.press)
    waiting_hyst -> idle (f.release)
    waiting_hyst -> pressed (waiting_hyst.c.leave)
    pressed -> idle (f.release)
    pressed -> panning (f.move)
    panning -> idle (f.release)
    idle -> zooming (f.wheel)
    zooming -> idle (zooming.cl.tick)
    zooming -> waiting_hyst (bg.press)
    */
  }

	Switch debugScene (no) {
    Component no
		Component yes {
	  	// Debug coordinates conversions
	  	Component scene {
		    Scaling scaling (1, 1, 0, 0)
		    zoom => scaling.sx, scaling.sy

		    Translation translation (0, 0)
		    xpan => translation.tx
		    ypan => translation.ty

		    FillColor fc (0, 255, 0)
		    FillOpacity fo (0.5)
		    Circle cDebug (300, 250, 4)
		  }

  		// Debug coordinates conversions
  		//   in screen coord system
  		FillColor green(0, 0, 255)
  		Circle cDebug (0, 0, 2)
  		f.move.x => cDebug.cx
  		f.move.y => cDebug.cy
  		//   in local coord system
  		f.move.x / zoom - xpan => scene.cDebug.cx
  		f.move.y / zoom - ypan => scene.cDebug.cy

      // Debug state machine
      TextPrinter tp
      fsm.state => tp.input
    }
  }

}