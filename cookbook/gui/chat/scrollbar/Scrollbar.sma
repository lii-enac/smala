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

// MDPC scrollbar, or
// M(t)DP(i)C scrollbar
// MDPC is a model for interactive graphical objects, which can be considered as a refinement of MVC.
// With MVC, it is notoriously difficult to cleanly separate the View and the Controller (see https://www.oracle.com/java/technologies/a-swing-architecture.html)
//  "We quickly discovered that this split didn't work well in practical terms because the view and controller parts of a component required a tight coupling (for example, it was very difficult to write a generic controller that didn't know specifics about the view). So we collapsed these two entities into a single UI (user-interface) object,"
// With MDPC, we applied the "separation of concerns" down to the MVC Controller, which enables designers to cleanly separate the View and the Controller
// MDPC could have been named MtDPiC:

// M = model - the abstraction. Here two values (low,hi) in range [0;1], and two operations (add to low and hi, including negative deltas). An _illustration_ of the Model is: 0--|--|---1
// t = transform - transforms the model into the display and the picking views. Typically a scaling that maps [0;1] to [min;max], a translation to place the scrollbar on the screen, and an orientation to select horizontal or vertical.
// D = display view - what the user sees, for a horizontal scrollbar: |<|==|    |=====|>| (left arrow, bg, thumb on top of the bg, right arrow)
// P = picking view - what the user actually manipulates, without seeing it:   |O|**|@@@|$$$$$|8| (0:left arrow, *:left-to-thumb, @:thumb, $:right-to-thumb, 8:right arrow)
// i = inverse transform - inverse-transforms user's actions back from the screen coordinates into the model coordinates
// C = controller - manages the interactive state, translates user's actions into model operations by relying on the inverse transform, triggers the model operations

// with this architecture, the scrollbar can be arbitrarily transformed:
// scaled by any amount, translated in any place, 90° rotation from vertical to horizontal, or any rotation (as in the example).
// thanks to the picking view and the inverse transform the controller code is simple and independent of the transform (e.g. it's the same for the horizontal and vertical scrollbar)

// see:
// Conversy, S., Barboni, E., Navarre, D., Palanque, P. Improving modularity of interactive software with the MDPC architecture. IFIP EIS 2007.
// 	     http://dx.doi.org/10.1007/978-3-540-92698-6_20
// Conversy, S. Improving Usability of Interactive Graphics Specification and Implementation with Picking Views and Inverse Transformations. In IEEE VL/HCC 2011.
// 	     http://dx.doi.org/10.1109/VLHCC.2011.6070392


use core
use base
use display
use gui

import gui.widgets.IWidget

import paging
import inverse_transform

// TODO: fix cairo bug with enter/leave when moving shape
// TODO: make it actually usable

_define_
Scrollbar(double _x, double _y, Process container, Process frame) {

  // Double x(0)
  // Double y(0)

  Translation pos (_x, _y)
  x aka pos.tx
  y aka pos.ty

  // ---------------
  // Model
  Component model {
    Double low  (0.75)
    Double high (0.9)
    // helper properties
    Double delta (0)
    high - low =:> delta
  }

  // model operations 
  AdderAccumulator add_low  (0, 0, 1.5)
  AdderAccumulator add_high (0, 0, 1.5)

  Incr incr_low  (0)
  Incr incr_high (0)


  // -----------------
  // Transform

  // The 'model' of the transform: a translation, a scaling, and a rotation  
  Component transform {
    Double tx (0)
    Double ty (0)
    Double  s (1)
    Double rot (0)

    // impl
    Double sina (0)
    Double cosa (0)
    cos(transform.rot * 3.1415 / 180.) =:> cosa
    sin(transform.rot * 3.1415 / 180.) =:> sina
  }

  Double width (10)
  Double height (10)
  //width aka this.width
  Double arrow_height(10)
//  arrow_height * 3 =: this.min_height
//  5 =: this.min_width 

  x =:> transform.tx
  y =:> transform.ty
  height - arrow_height =:> transform.s

  // cancel translation from IWidget, since we need a Rotation before
  Translation _cancel_iwidget_tr(0,0)
  -x =:> _cancel_iwidget_tr.tx
  -y =:> _cancel_iwidget_tr.ty

  // the transform implemented as a graphical transform to generate the Display and Picking views
  Rotation    rot(0,0,0)
  Translation tr(0,0)
  Scaling     sc(1,1, 0,0)

  transform.rot =:> rot.a
  transform.tx /*- width/2*/  =:> tr.tx
  transform.ty /*- height/2*/ =:> tr.ty
  transform.tx /*- width/2*/  =:> rot.cx
  transform.ty /*- height/2*/ =:> rot.cy
  transform.s   =:> sc.sy



  // -----------------
  // Display view


  NoOutline _

  // the display view is the same regardless of the status of the interaction
  // it's thus a single Component
  Component display_view {
    
    FillColor   _ (255,255,255) // white
    Rectangle   more_arrow_bg (0,0,1,1,0,0) // ^
    FillColor   _ (200,200,200) // gray
    Rectangle   bg (0,0,1,1,0,0)            // []
    FillColor   _ (255,255,255) // white
    Rectangle   less_arrow_bg (0,0,1,1,0,0) // v

    // thumb on top
    FillColor   _ (0x32,0x32,0x32) // darkgray
    Rectangle   thumb (0,0,1,1,0,0)         // =

    // 'one-way constraint' or data-flow of position/size of each zone for a regular scrollbar display layout
    // transforms the model into a display view (background + arrows + thumb)

    //                                       x, width
                                         width =:> more_arrow_bg.width, bg.width, thumb.width, less_arrow_bg.width
                                   0 - width/2 =:> more_arrow_bg.x, bg.x, thumb.x, less_arrow_bg.x
    //                                       y, height
                                  0 /*- height/2*/ =:> more_arrow_bg.y 
                    arrow_height / transform.s =:> more_arrow_bg.height
        more_arrow_bg.y + more_arrow_bg.height =:> bg.y
                                             1 =:> bg.height
                              bg.y + bg.height =:> less_arrow_bg.y
                    arrow_height / transform.s =:> less_arrow_bg.height

                         (1-model.high) + bg.y =:> thumb.y
                                   model.delta =:> thumb.height

  }


  // -----------------
  // Picking view
  Int pick_xoffset(0) // display the picking view to the right of the display view by xoffset for demonstration purpose

  // the picking view has a state that depends on the status of the interaction (see controller)
  // so let's make it a Switch
  Switch picking_view (initial) {

    Component initial {

      NoFill _
      NoOutline _
      PickFill _
      NoPickOutline _
      
      FillColor _ (0,255,0) //green
      Rectangle more_arrow (0,0,1,1,0,0)  // ^
      FillColor _ (0,255,255) //cyan
      Rectangle more_bg (0,0,1,1,0,0)     // ||
      FillColor _ (255,255,0) //yellow
      Rectangle less_bg (0,0,1,1,0,0)     // ||
      FillColor _ (255,0,0) //red
      Rectangle less_arrow (0,0,1,1,0,0)  // v

      // thumb on top
      FillColor _ (255,0,255) //purple
      Rectangle thumb (0,0,1,1,0,0)       // =
     
      // 'one-way constraint' or data-flow of position/size of each zone for a classical scrollbar picking layout in idle state
      // transforms the model into a picking view (background + arrows + thumb)     

                                         width =:> more_arrow.width, more_bg.width, thumb.width, less_bg.width, less_arrow.width
                    0 - width/2 + pick_xoffset =:> more_arrow.x,     more_bg.x,     thumb.x,     less_bg.x,     less_arrow.x

                                             0 =:> more_arrow.y
                    arrow_height / transform.s =:> more_arrow.height // ^
              more_arrow.y + more_arrow.height =:> more_bg.y
                              (1 - model.high) =:> more_bg.height    // ||
                 more_bg.y +    more_bg.height =:> thumb.y           //
                                   model.delta =:> thumb.height      // =
                   thumb.y +      thumb.height =:> less_bg.y         //
                                     model.low =:> less_bg.height    // ||
                 less_bg.y +    less_bg.height =:> less_arrow.y      //
                    arrow_height / transform.s =:> less_arrow.height // v            
            
    }

    Component hyst {
      // the hysteresis circle is in screen space, so we inverse the graphical transform
      // should be LoadIdentity(): it would be more efficient and on par with the semantics

      // Scaling     sc(1,1, 0,0)
      // Translation tr(0,0)
      // Rotation    rot(0,0,0)
      
      //  -transform.rot =:> rot.a
      //  -transform.tx  =:> tr.tx
      //  -transform.ty  =:> tr.ty
      // 1/transform.s   =:> sc.sy

      //   transform.tx  =:> rot.cx
      //   transform.ty  =:> rot.cy

      Identity _

      NoFill _
      NoOutline _
      PickFill _
      NoPickOutline _

      FillColor fc (255, 255, 0)
      Circle    c (0,0, 5)                   // °
    }

    Component dragging {
      // vv maybe useless since we use a transformed analytical computation anyway, keep it for explanation/debugging purpose
      //FillOpacity _(0.5)

      NoFill _
      NoOutline _
      PickFill _
      NoPickOutline _

      FillColor _ (0,255,0) // green
      Rectangle upper_limit (0,0,1,1,0,0)    // || (!)
      FillColor _ (0,255,255) // cyan
      Rectangle dragging_zone (0,0,1,1,0,0)  // || (=)
      FillColor _ (255,0,0) // red
      Rectangle lower_limit (0,0,1,1,0,0)    // || (!)

      Double pick_offset_in_model (0)
      Double zero_in_model (0)   // does not work very well (?)
      Double win_height_in_model (0) // idem
      
      // pick_offset in model coordinates
      inverse_transform _ (transform, picking_view.hyst.c.cx, picking_view.hyst.c.cy, pick_offset_in_model)
      Double zero(0)
      inverse_transform _ (transform, zero, zero, zero_in_model) // FIXME? not working as planned
      inverse_transform _ (transform, container.height, container.height, win_height_in_model) // FIXME? not working as planned
     
      // 'one-way constraint' or data-flow of position/size for a classical scrollbar picking layout in dragging state
                                          // x, width
                                         width =:> upper_limit.width, dragging_zone.width, lower_limit.width
                    0 - width/2 + pick_xoffset =:> upper_limit.x, dragging_zone.x, lower_limit.x

                                          // y, height
                                 zero_in_model =:> upper_limit.y

           picking_view.initial.more_bg.y
        + (pick_offset_in_model - picking_view.initial.thumb.y)
        -  upper_limit.y                       =:> upper_limit.height

          upper_limit.y +   upper_limit.height =:> dragging_zone.y
         1 - picking_view.initial.thumb.height =:> dragging_zone.height
        dragging_zone.y + dragging_zone.height =:> lower_limit.y
	         win_height_in_model - lower_limit.y =:> lower_limit.height
      
      // ^^ maybe useless since we use a transformed analytical computation anyway, keep it for explanation/debugging purpose
      
      // A picking view would require that the upper and lower zone fill-up the entire screen
      // this is both inefficient (fill-rate!!), but also requires to compute coordinates according to the current transformation...
      

      // transformed analytical computation
      // ...so use a transformed analytical computation instead
      // this computes the position of the initial press in the model coordinate system and assess if it's between the boundaries
      
      Double cursor_in_model (0)
      inverse_transform _(transform, frame.move.x, frame.move.y, cursor_in_model)

      Bool higher (0)
      cursor_in_model > dragging_zone.y + dragging_zone.height =:> higher
      Bool lower (0)
      cursor_in_model < dragging_zone.y =:> lower
      
      //Bool inside (0) // don't use it for now, to keep "compatibility/similarity" with picking view code
      //!higher && !lower =:> inside
      
      //TextPrinter tp
      //"inside " + inside + " lower " + lower + " higher " + higher =:> tp.input
    } 
  }


  // -----------------
  // Controller == management of interactive state, with an FSM

  DoubleProperty lastv (0)

  FSM fsm {

    State idle {
      "initial" =: picking_view.state
      Double dv (0)
      Double fakex(0)
      Double fakey(0)
      frame.wheel.dy / transform.s => dv
      dv<0 ?
        (-dv < (model.low) ? dv : -model.low)
        : (dv < (1-model.high) ? dv : 1-model.high) =:> fakex

      //inverse_transform iv(transform, fakex, fakey, dv)
      //TextPrinter tp2
      AssignmentSequence asw(0) {
        //"asw" =: tp2.input
        //dv + model.low  =: model.low
        //dv + model.high =: model.high
        //dv<0 ? (-dv > (model.low) ? dv : -model.low) + model.low =: model.low
        //dv>0 ? (dv < (1-model.high) ? dv : 1-model.high) + model.high =: model.high
        fakex + model.low =: model.low
        fakex + model.high =: model.high
      }

      frame.wheel -> asw
      //TextPrinter tp
      //fakey =:> tp.input
      //"dw dv " + toString(dv) =:> tp.input

    }

    State onelining_up {
      paging p(model)
      Double dv (0.1)
      dv =: p.dv
    }

    State onelining_down {
      paging p(model)
      Double dv (-0.1)
      dv =: p.dv
    }

    State paging_up {
      paging p(model)
      model.delta =: p.dv
    }

    State paging_down {
      paging p(model)
     -model.delta =: p.dv
    }

    State paging_still // FIXME, enter is not working with cairo when moving shapes

    State waiting_hyst {
      "hyst" =: picking_view.state

      frame.press.x =: picking_view.hyst.c.cx
      frame.press.y =: picking_view.hyst.c.cy

      // initialize inverse transform from user actions to model operations
      inverse_transform iv(transform, frame.press.x, frame.press.y, lastv)
    }

    State dragging {
      "dragging" =: picking_view.state
      
      // inverse transform from user actions to model operations
      Double dv (0)
      Double v (0)
      AssignmentSequence as(0) {
        // compute inverse and delta
        inverse_transform iv(transform, frame.move.x, frame.move.y, v)
        - (v - lastv) =: dv
        // remember last pos
        v =: lastv
        // transform actions into operations on the model, and apply them
        dv + model.low  =: model.low
        dv + model.high =: model.high
      }
      frame.move -> as
      //TextPrinter tp
      //frame.move.y => tp.input
      //"movey " + toString(dv) => tp.input
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


    // transitions                                                              //  UAN User-Action Notation https://www.semanticscholar.org/paper/The-UAN%3A-A-User-Oriented-Representation-for-Direct-Hartson-Siochi/97a593273fca9460ce05f32031896b752de4dcb9/figure/16

    // lining/paging                                                            
             idle -> onelining_up   (picking_view.initial.more_arrow.press)     //  [^]v
             idle -> onelining_down (picking_view.initial.less_arrow.press)     //  [v]v

     onelining_up -> idle           (frame.release)
   onelining_down -> idle           (frame.release)

             idle -> paging_up      (picking_view.initial.more_bg.press)        //  [||]v
             idle -> paging_down    (picking_view.initial.less_bg.press)        //  [||]v

      paging_down -> paging_up      (picking_view.initial.more_bg.enter)        // ~[||]
        paging_up -> paging_down    (picking_view.initial.less_bg.enter)        // ~[||]
      
        paging_up -> paging_still   (picking_view.initial.thumb.enter)          // ~[=]
      paging_down -> paging_still   (picking_view.initial.thumb.enter)          // ~[=]

     paging_still -> paging_up      (picking_view.initial.more_bg.enter)        // ~[||]
     paging_still -> paging_down    (picking_view.initial.less_bg.enter)        // ~[||]

        paging_up -> idle           (frame.release)                                 //  ^
     paging_still -> idle           (frame.release)                                 //  ^
      paging_down -> idle           (frame.release)                                 //  ^

    // dragging
             idle -> waiting_hyst   (picking_view.initial.thumb.press)          //  [=]v 
     waiting_hyst -> idle           (frame.release)                                 //  ^
     waiting_hyst -> dragging       (picking_view.hyst.c.leave)                 //  [o]~   // FIXME does not work with fast movements

         dragging -> idle           (frame.release)                                 //  ^

         // with analytical computation (see below an alternative version with picking views)
         dragging -> in_upper_zone  (picking_view.dragging.lower.true)          // ~[-]   // strange should be "higher" ?!
    in_upper_zone -> dragging       (picking_view.dragging.lower.false)         // ~[=]

         dragging -> in_lower_zone  (picking_view.dragging.higher.true)         // ~[_]   // strange should be "lower" ?!
    in_lower_zone -> dragging       (picking_view.dragging.higher.false)        // ~[=]

    in_lower_zone -> idle           (frame.release)                                 //  ^
    in_upper_zone -> idle           (frame.release)                                 //  ^
  }
}


    // with picking view, but would require infinitely extending picking zones :-/
    //      dragging -> in_upper_zone  (picking_view.dragging.upper_limit.enter)   // ~[-]   // FIXME no message/warning when path is erroneous
    // in_upper_zone -> dragging       (picking_view.dragging.dragging_zone.enter) // ~[=]

    //      dragging -> in_lower_zone  (picking_view.dragging.lower_limit.enter)   // ~[_]
    // in_lower_zone -> dragging       (picking_view.dragging.dragging_zone.enter) // ~[=]