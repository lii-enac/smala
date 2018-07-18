use core
use base
use gui

/*
 * Computes pan and mouse centered zoom of a component transformed by a scaling then a translation.
 * Works by connecting interface members to these scaling and translation.
 */
_define_
PanAndZoom (Component f) {

  /*----- interface -----*/
  Double zoom (1)
  Double xpan (0)
  Double ypan (0)
  /*--- end interface ---*/

  Component tempPos {
    DoubleProperty x (0)
    DoubleProperty y (0)
  }

  // Pan management
  FSM fsm {
    State idle

    // Hysteresis version
    // With the hysteresis version, it's better to have a scene component (graphical elements 
    // along with their transforms) and a fsm in distinct branches of the components tree 
    // in order to keep the hysteresis circle drawing simple in the screen coordinates system
    // (with none of the transforms applied to the scene component)
    State waiting_hyst {
      FillColor fcwh (255, 0, 0)
      Circle c (0, 0, 20)
      f.press.x =: c.cx
      f.press.y =: c.cy
      0 =: tempPos.x
      0 =: tempPos.y
    }

    State panning {
      // init tempPos when entering the state
      f.press.x =: tempPos.x
      f.press.y =: tempPos.y

      // define sequence of assignments to activate on mouse move
      AssignmentSequence sequence (1) {
        xpan + (f.move.x - tempPos.x) / zoom =: xpan
        ypan + (f.move.y - tempPos.y) / zoom =: ypan
        f.move.x =: tempPos.x
        f.move.y =: tempPos.y
      }
      f.move -> sequence
    }

    State zooming {
      // init a clock to leave this state after a timeout without interaction
      Clock cl (1000)
      
      // compute scale factor to modify zoom (data flow)
      Pow p (1.01, 0)
      Double scaleFactor (1)
      f.wheel.dy => p.exponent
      p.result => scaleFactor

      Double p0x (0) // mouse x in local coord system before zoom
      Double p0y (0) // mouse y in local coord system before zoom

      // define sequence of assignments to activate on mouse wheel
      AssignmentSequence sequence (0) {
        // remember mouse pos in local coord system before zoom
        // (as a local coord it won't change after zoom is applied)
        f.move.x / zoom - xpan =: p0x
        f.move.y / zoom - ypan =: p0y
        // apply new zoom
        zoom * scaleFactor =: zoom
        // After the zoom is applied, p0 has become p1 = p0 * scaleFactor
        // So to make believe that zoom is mouse centered, we must apply
        // a new translation (p0 - p1) taking the scaleFactor into account
        // i.e replace pan by (pan + p0 - p1)/scaleFactor
        (xpan + p0x * (1 - scaleFactor)) / scaleFactor =: xpan
        (ypan + p0y * (1 - scaleFactor)) / scaleFactor =: ypan
      }
      f.wheel -> sequence, cl
    }

    // Transitions for basic version
    idle -> panning (f.press)
    panning -> idle (f.release)
    idle -> zooming (f.wheel)
    zooming -> idle (zooming.cl.tick)
    zooming -> panning (f.press)
    
    /*
    // Transitions for hysteresis version
    idle -> waiting_hyst (f.press)
    idle -> zooming (f.wheel)
    waiting_hyst -> idle (f.release)
    waiting_hyst -> panning (waiting_hyst.c.leave)
    panning -> idle (f.release)
    zooming -> idle (zooming.cl.tick)
    zooming -> waiting_hyst (f.press)
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