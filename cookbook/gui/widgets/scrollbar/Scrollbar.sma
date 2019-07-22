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
// transform = transforms the model into the display and the picking views
// inverse transform = inverse transforms user's actions and _performs_ the translation into model operations

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

  // ---------------
  // model
  Component model {
    Double low  (0.75)
    Double high (0.9)
    // helper properties
    Double delta (0)
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
     Double ty (100)
     Double  s (400)
     //DoubleProperty rot (45) // not yet
  }

  // -----------------
  // display view
  Component display_view {

    FillColor    ac (255,255,255) // white
    Rectangle   more_arrow (0,0,100,100,0,0) // ^
    Rectangle   less_arrow (0,0,100,100,0,0) // v
    FillColor   mtc (200,200,200) // gray
    Rectangle    bg (0,0,100,100,0,0)        // []
    FillColor   mtp (150,150,255) // grayblue
    Rectangle thumb (0,0,100,100,0,0)        // =

    // 'one-way constraint' or data-flow of position/size of each zone for a regular scrollbar display layout

                                                  bg.x = 200.0                                 // FIXME? "200" marche pas, pas de message
                                                  bg.x =:> thumb.x, more_arrow.x, less_arrow.x

                              bg.y - more_arrow.height =:> more_arrow.y
	                            bg.y +         bg.height =:> less_arrow.y


     // transformation of model into display view for background and thumb

                      transform.ty + more_arrow.height =:> bg.y
                                           transform.s =:> bg.height

     (1-model.high) * transform.s + transform.ty
                                   + more_arrow.height =:> thumb.y
                  (model.high-model.low) * transform.s =:> thumb.height
  }

  // -----------------
  // picking view
  // a picking view has a state that depends on the status of the interaction (see controller)
  Switch picking_view (initial) {

    Component initial {
      FillColor mac (0,255,0)//green
      Rectangle more_arrow (0,0,100,100,0,0)  // ^
      FillColor mtb (0,255,255)//cyan
      Rectangle more_bg (0,0,100,100,0,0)     // ||
      FillColor mtc (255,0,255)//purple
      Rectangle thumb (0,0,100,100,0,0)       // =
      FillColor ltc (255,255,0)//yellow
      Rectangle less_bg (0,0,100,100,0,0)     // ||
      FillColor lac (255,0,0)//red
      Rectangle less_arrow (0,0,100,100,0,0)  // v
     
        // 'one-way constraint' or data-flow of position/size of each zone for a regular scrollbar picking layout

                                            // =:> more_arrow.y  // ^
              more_arrow.y + more_arrow.height =:> more_bg.y     // ||
                 more_bg.y + more_bg.height    =:> thumb.y       // =
                 thumb.y   +   thumb.height    =:> less_bg.y     // ||
                 less_bg.y + less_bg.height    =:> less_arrow.y  // v  

	      // transformation of model into picking view for background and thumb

                                  transform.ty =:> picking_view.initial.more_arrow.y    // ^     
                (1 - model.high) * transform.s =:> picking_view.initial.more_bg.height  // ||
                     model.delta * transform.s =:> picking_view.initial.thumb.height    // =
                       model.low * transform.s =:> picking_view.initial.less_bg.height  // ||
                                                                                        // v
    }

    Component hyst {
      FillColor fc (255, 0, 0)
      Circle    c (0,0, 5)                          // °
      Int       offset (0)
    }

    Component dragging {
      FillColor mac (255,0,0) // r
      Rectangle upper_limit (0,0,100,100,0,0)    // || (!)
      FillColor mtc (0,255,0) // g
      Rectangle dragging_zone (0,0,100,100,0,0)  // || (=)
      FillColor mtp (0,0,255) // b
      Rectangle lower_limit (0,0,100,100,0,0)    // || (!)
     
        // 'one-way constraint' or data-flow of position/size for a regular scrollbar picking layout

	       upper_limit.y +   upper_limit.height =:> dragging_zone.y
       dragging_zone.y + dragging_zone.height =:> lower_limit.y
	            f.height -        lower_limit.y =:> lower_limit.height

          picking_view.initial.more_bg.y
	     + (picking_view.hyst.offset - picking_view.initial.thumb.y)
	     -  picking_view.dragging.upper_limit.y =: picking_view.dragging.upper_limit.height

        transform.s
          - picking_view.initial.thumb.height =: picking_view.dragging.dragging_zone.height
    } 
  }

  // -----------------
  // controller = management of interactive state with an FSM

  IntProperty lasty (0)

  FSM fsm {

    State idle {
      // change picking state
      TextProperty pv_state ("initial")
      pv_state =: picking_view.state
    }

    State onelining_up {
      paging p(model, f)
      Double dv (0.1)
      dv =: p.dv
    }

    State onelining_down {
      paging p(model, f)
      Double dv (-0.1)
      dv =: p.dv
    }

    State paging_up {
      paging p(model, f)
      model.delta =: p.dv
    }

    State paging_down {
      paging p(model, f)
     -model.delta =: p.dv
    }

    State paging_still

    State waiting_hyst {
      // change picking state
      TextProperty pv_state ("hyst")
       pv_state =: picking_view.state

      f.press.x =: picking_view.hyst.c.cx
      f.press.y =: picking_view.hyst.c.cy

      f.press.y =: picking_view.hyst.offset
      f.press.y =: lasty
    }

    State dragging {
      // change picking state
      TextProperty pv_state ("dragging")
      pv_state =: picking_view.state
      
      // inverse transform from user actions to model operations
      Component inverse_transform {
        Double dv (0)
        Double dy (0)
        Double y (0)

        // FIXME: dependency with view layout, clamping of event coordinate should be implemented in scene graph ?
        // or receive dragging_zone.move and leave only
        clamp clamp_ (f.move.y, picking_view.dragging.dragging_zone.y, picking_view.dragging.lower_limit.y, y)

        AssignmentSequence as(0) {
          lasty - y =: dy      // should be (lasty - transform.ty) - (f.move.y - transform.ty) =:> dy , but transform.ty self-cancels
          dy / transform.s =: dv

          // apply to model
          dv + model.low  =: model.low
          dv + model.high =: model.high

          // remember last pos
          y =: lasty
        }
        f.move -> as
      }
    }
    
    State in_upper_zone {
      AssignmentSequence _(0) {
        1 - model.delta =: model.low
        1 =: model.high
      }
    }

    State in_lower_zone {
      AssignmentSequence _(0) {
        0 =: model.low
        model.delta =: model.high
      }
    }

    // transitions
                                                                                //   UAN User-Action Notation https://www.semanticscholar.org/paper/The-UAN%3A-A-User-Oriented-Representation-for-Direct-Hartson-Siochi/97a593273fca9460ce05f32031896b752de4dcb9/figure/16
             idle -> onelining_up   (picking_view.initial.more_arrow.press)     //  [^]v
             idle -> onelining_down (picking_view.initial.less_arrow.press)     //  [v]v

     onelining_up -> idle           (f.release)
   onelining_down -> idle           (f.release)

             idle -> paging_up      (picking_view.initial.more_bg.press)        //  [||]v
             idle -> paging_down    (picking_view.initial.less_bg.press)        //  [||]v

      paging_down -> paging_up      (picking_view.initial.more_bg.enter)        // ~[||]
        paging_up -> paging_down    (picking_view.initial.less_bg.enter)        // ~[||]
      
        paging_up -> paging_still   (picking_view.initial.thumb.enter)          // ~[=]
      paging_down -> paging_still   (picking_view.initial.thumb.enter)          // ~[=]

     paging_still -> paging_up      (picking_view.initial.more_bg.enter)        // ~[||]
     paging_still -> paging_down    (picking_view.initial.less_bg.enter)        // ~[||]

        paging_up -> idle           (f.release)                                 //  ^
     paging_still -> idle           (f.release)                                 //  ^
      paging_down -> idle           (f.release)                                 //  ^

             idle -> waiting_hyst   (picking_view.initial.thumb.press)          //  [=]v 
     waiting_hyst -> idle           (f.release)                                 //  ^
     waiting_hyst -> dragging       (picking_view.hyst.c.leave)                 //  [o]~   // FIXME does not work with fast movements

         //dragging -> dragging       (f.move)                                  //  ~    // in state
         dragging -> idle           (f.release)                                 //  ^

         dragging -> in_upper_zone  (picking_view.dragging.upper_limit.enter)   // ~[-]   // FIXME no message/warning when path is erroneous
    in_upper_zone -> dragging       (picking_view.dragging.dragging_zone.enter) // ~[=]

         dragging -> in_lower_zone  (picking_view.dragging.lower_limit.enter)   // ~[_]
    in_lower_zone -> dragging       (picking_view.dragging.dragging_zone.enter) // ~[=]

    in_lower_zone -> idle           (f.release)                                 //  ^
    in_upper_zone -> idle           (f.release)                                 //  ^
  }
  //fsm.state  =:> tp.input
}
