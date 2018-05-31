/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *		Stéphane Conversy <stephane.conversy@enac.fr>
 *
 */

use core
use base
use gui

_action_
null_func (Component c)
%{
{
}
%}

_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 600)
  Exit ex (0, 1)
  f.close -> ex

  NativeAction na_null_func (null_func, 1)

  FillColor fc(0, 0, 0)
  //GHomography t()
  Translation tr(0,0)
  Scaling sc(1,1,0,0)
  //GRectangle mobile(0, 0, 100, 70, 10, 10)
  Circle mobile(100, 100, 40)

  mobile.press->na_null_func /* bug */

  Component offset {
  	    DoubleProperty x (0)
  	    DoubleProperty y (0)
  }

  Component mouse {
  	    DoubleProperty dx (0)
  	    DoubleProperty dy (0)
  	    DoubleProperty xlast (0)
  	    DoubleProperty ylast (0)
	    f.move.x - xlast => dx
	    f.move.y - ylast => dy
	    f.move.x => xlast
	    f.move.y => ylast
	    
	    //TextPrinter dp
	    //dx => dp.input
  }

  Switch sw (idle) {
    Component idle {
    }
    Component waiting_hyst {
      FillColor fcwh(255, 0, 0)
      Circle c(0,0, 20)
      f.press.x - tr.tx =: c.cx
      f.press.y - tr.ty =: c.cy
      //f.press.x =: offset.x
      //f.press.y =: offset.y
    }
    Component dragging {
    	      /*f.press.x - mobile.x =: offset.x
	      f.press.y - mobile.y =: offset.y
	      f.move.x - offset.x => mobile.x
	      f.move.y - offset.y => mobile.y*/

    	      //f.press.x - tr.tx =: offset.x
	      //f.press.y - tr.ty =: offset.y

    	      sw.waiting_hyst.c.cx =: offset.x
	      sw.waiting_hyst.c.cy =: offset.y
	      f.move.x - offset.x => tr.tx
	      f.move.y - offset.y => tr.ty

	      //mouse.dx + tr.tx => tr.tx
	      //mouse.dy + tr.ty => tr.ty

    }
  }

  Switch s(true) {
    Component true {}
    Component false {}
  }

  FSM fsm {
    State idle
    State waiting_hyst
    State dragging
    
    idle->waiting_hyst (mobile.press)
    waiting_hyst->idle (f.release)
    /*waiting_hyst->dragging (f.move)*/
    waiting_hyst->dragging (sw.waiting_hyst.c.leave)
    dragging->idle (f.release)
    dragging->dragging (f.move)
  }

  fsm.state => sw.state
  TextPrinter tp
  sw.state=>tp.input
}

run root
run syshook
