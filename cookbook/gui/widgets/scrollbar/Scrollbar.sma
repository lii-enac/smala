/*
 *	MDPC scrollbar
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Stéphane Conversy <stephane.conversy@enac.fr>
 *
 */

// MDPC scrollbar

// M = model - the abstract data, here two values in range [0;1]
// D = display view - what the user actually sees
// P = picking view - what the user actually manipulates, without seeing it
// C = controller - manages interactive state and _triggers_ the translation of user's actions into model operations
// transformation = transforms the model into the display and the picking views
// inverse transformation = inverse-transforms user's actions and _performs_ the translation into model operations

// see:
// Conversy, S., Barboni, E., Navarre, D., Palanque, P. Improving modularity of interactive software with the MDPC architecture. IFIP EIS 2007.
// 	     http://dx.doi.org/10.1007/978-3-540-92698-6_20
// Conversy, S. Improving Usability of Interactive Graphics Specification and Implementation with Picking Views and Inverse Transformations. In IEEE VL/HCC 2011.
// 	     http://dx.doi.org/10.1109/VLHCC.2011.6070392

use core
use base
use gui

import paging
import clamp

_define_
Scrollbar(Process f) {
  //TextPrinter tp
  //f.move.y =:> tp.input

  // ---------------
  // model
  Component model {
    DoubleProperty low  (0.75)
    DoubleProperty high (0.9)
    // helper properties
    DoubleProperty delta (0)
    high - low =:> delta
  }
  //model.low =:> tp.input

  // model operations 
  AdderAccumulator add_low  (0, 0, 1.5)
  AdderAccumulator add_high (0, 0, 1.5)

  Incr incr_low  (0)
  Incr incr_high (0)

  // -----------------
  // transform
  Component transform {
     DoubleProperty ty (100)
     DoubleProperty  s (400)
     //DoubleProperty rot (45) // not yet
  }

  // -----------------
  // display view
  Component display_view {
     FillColor   mac (255,255,255) // white
     Rectangle   more_arrow (0,0,100,100,0,0) // ^
     Rectangle   less_arrow (0,0,100,100,0,0) // v
     FillColor   mtc (200,200,200) // gray
     Rectangle    bg (0,0,100,100,0,0)        // []
     FillColor   mtp (150,150,255) // grayblue
     Rectangle thumb (0,0,100,100,0,0)        // =

     // 'one-way constraint' or data-flow of position/size of each zone for a regular scrollbar layout

                                                  bg.x = 200.0                                 // FIXME? "200" marche pas, pas de message
                                                  bg.x =:> thumb.x, more_arrow.x, less_arrow.x

                              bg.y - more_arrow.height =:> more_arrow.y
	                            bg.y +         bg.height =:> less_arrow.y


     // transformation of model into display view for background and thumb

                      transform.ty + more_arrow.height =:> bg.y
                                           transform.s =:> bg.height     
     (1-model.high) * transform.s + transform.ty + 100 =:> thumb.y
                  (model.high-model.low) * transform.s =:> thumb.height
  }

  // -----------------
  // picking view
  Switch picking_view (initial) {

    Component initial {
        FillColor mac (0,255,0)//green
        Rectangle more_arrow (0,0,100,100,0,0)  // ^
        FillColor mtb (0,255,255)//cyan
        Rectangle more_bg (0,0,100,100,0,0)     // []
        FillColor mtc (255,0,255)//purple
        Rectangle thumb (0,0,100,100,0,0)       // =
        FillColor ltc (255,255,0)//yellow
        Rectangle less_bg (0,0,100,100,0,0)     // []
        FillColor lac (255,0,0)//red
        Rectangle less_arrow (0,0,100,100,0,0)  // v
     
        // 'one-way constraint' or data-flow of position/size of each zone for a regular scrollbar layout

                                            // =:> more_arrow.y  // ^
              more_arrow.y + more_arrow.height =:> more_bg.y     // []
                 more_bg.y + more_bg.height    =:> thumb.y       // =
                 thumb.y   +   thumb.height    =:> less_bg.y     // []
                 less_bg.y + less_bg.height    =:> less_arrow.y  // v  

	      // transformation of model into display view for background and thumb

                                  transform.ty =:> picking_view.initial.more_arrow.y    // ^     
                (1 - model.high) * transform.s =:> picking_view.initial.more_bg.height  // []
        (model.high - model.low) * transform.s =:> picking_view.initial.thumb.height    // =
                     (model.low) * transform.s =:> picking_view.initial.less_bg.height  // []
                                                                                        // v
     }

    Component hyst {
        //Translation t(50,50)
        FillColor fc (255, 0, 0)
        Circle c (0,0, 5)                          // °

	      IntProperty offset (0)
    }

    Component dragging {
        FillColor mac (255,0,0) // r
        Rectangle upper_limit (0,0,100,100,0,0)    // [] (!)
        FillColor mtc (0,255,0) // g
        Rectangle dragging_zone (0,0,100,100,0,0)  // [] (=)
        FillColor mtp (0,0,255) // b
        Rectangle lower_limit (0,0,100,100,0,0)    // [] (!)
     
        // 'one-way constraint' or data-flow of position/size for a regular scrollbar layout

	      upper_limit.y +   upper_limit.height =:> dragging_zone.y
      dragging_zone.y + dragging_zone.height =:> lower_limit.y
	           f.height -        lower_limit.y =:> lower_limit.height

          picking_view.initial.more_bg.y
	     + (picking_view.hyst.offset - picking_view.initial.thumb.y)
	     -  picking_view.dragging.upper_limit.y =: picking_view.dragging.upper_limit.height

        transform.s - picking_view.initial.thumb.height =: picking_view.dragging.dragging_zone.height
    } 
  }

  // -----------------
  // controller = FSM + switch

  IntProperty lasty (0)

  Switch sw (idle) {

    Component idle {
      TextProperty pv_state ("initial")
      pv_state =: picking_view.state
    }

    Component onelining_up {
    	 paging p(model, f)
	     Double dv (0.1)
            dv =: p.dv
    }

    Component onelining_down {
    	 paging p(model, f)
	     Double dv (-0.1)
            dv =: p.dv
    }

    Component paging_up {
    	  paging p(model, f)
        model.delta =: p.dv
    }

    Component paging_down {
    	  paging p(model, f)
        -model.delta =: p.dv
    }

    Component paging_still {}

    Component waiting_hyst {
      TextProperty pv_state ("hyst")
       pv_state =: picking_view.state

	    //f.press.y =:> tp.input

      f.press.x =: picking_view.hyst.c.cx
      f.press.y =: picking_view.hyst.c.cy

      f.press.y =: picking_view.hyst.offset
      f.press.y =: lasty
    }

    Component dragging {
      TextProperty pv_state ("dragging")
      pv_state =: picking_view.state
      
      /*DoubleProperty save_low (0)
      DoubleProperty save_high (0)
      model.low =: save_low
      model.high =: save_high*/

      // inverse transformation
      Component inverse_transform {
      	 // first setup dataflow to translate from user's actions to model operations
      	 Double dv (0)
	       //Double zero (0)

	       //  simulate dv + model.low =:> model.low and avoid cycle
	                     0 =:  add_low.result
	             model.low =:  add_low.input
                      dv =:> add_low.input
          add_low.result =:> model.low

	       //  simulate dv + model.high =:> model.high and avoid cycle
	                     0 =:  add_high.result
	            model.high =:  add_high.input
                      dv =:> add_high.input
         add_high.result =:> model.high

      	 DoubleProperty dy (0)
      	 //DoubleProperty cldv (0)

	       // actual inverse transformation
	       DoubleProperty y (0)
	       // FIXME, dependance with view layout, clamping of event coordinate should be implemented in scene graph ?
	       // or receive dragging_zone.move and leave only
	       //f.move.y =:> y
	       clamp clamp_ (f.move.y, picking_view.dragging.dragging_zone.y, picking_view.dragging.lower_limit.y, y)

               lasty - y =:> dy      // should be (lasty - transform.ty) - (f.move.y - transform.ty) =:> dy , but transform.ty self-cancels
	      dy / transform.s =:> dv

        //save_low + dv =:> model.low
        //save_high + dv =:> model.high

        //dy =:> tp.input

	       //            cldv =:> dv
         //(cldv > 1-model.high) ? 1-model.high : cldv =:> dv

	       // update interaction state
	       //f.move.y =:> lastY
	       y =:> lasty
      }
    }
    
    Component in_upper_zone {
    }

    Component in_lower_zone {
    }

  }

  FSM fsm {
    State idle

    State onelining_up
    State onelining_down

    State paging_up
    State paging_still
    State paging_down

    State waiting_hyst
    State dragging
    State in_upper_zone
    State in_lower_zone
    

             idle -> onelining_up   (picking_view.initial.more_arrow.press)     // ^
             idle -> onelining_down (picking_view.initial.less_arrow.press)     // v

     onelining_up -> idle           (f.release)
   onelining_down -> idle           (f.release)

             idle -> paging_up      (picking_view.initial.more_bg.press)        // [] *
             idle -> paging_down    (picking_view.initial.less_bg.press)        // [] *

      paging_down -> paging_up      (picking_view.initial.more_bg.enter)        // [] <-
        paging_up -> paging_down    (picking_view.initial.less_bg.enter)        // [] <-
      
        paging_up -> paging_still   (picking_view.initial.thumb.enter)          //  = <-
      paging_down -> paging_still   (picking_view.initial.thumb.enter)          //  = <-

     paging_still -> paging_up      (picking_view.initial.more_bg.enter)        // [] <-
     paging_still -> paging_down    (picking_view.initial.less_bg.enter)        // [] <-

        paging_up -> idle           (f.release)                                 // °
     paging_still -> idle           (f.release)                                 // °
      paging_down -> idle           (f.release)                                 // °

             idle -> waiting_hyst   (picking_view.initial.thumb.press)          // o * 
     waiting_hyst -> idle           (f.release)                                 // °
     waiting_hyst -> dragging       (picking_view.hyst.c.leave)                 // o ->   // FIXME does not work with fast movements

         dragging -> dragging       (f.move)                                    // <->
         dragging -> idle           (f.release)                                 // °

         dragging -> in_upper_zone  (picking_view.dragging.upper_limit.enter)   // _ <-   // FIXME no message/warning when path is erroneous
    in_upper_zone -> dragging       (picking_view.dragging.dragging_zone.enter) // = <-

         dragging -> in_lower_zone  (picking_view.dragging.lower_limit.enter)   // - <-
    in_lower_zone -> dragging       (picking_view.dragging.dragging_zone.enter) // = <-

    in_lower_zone -> idle           (f.release)                                 // *^
    in_upper_zone -> idle           (f.release)                                 // *^
  }

  fsm.state =:> sw.state // FIXME: no check that state names match
  //sw.state  =:> tp.input
}
