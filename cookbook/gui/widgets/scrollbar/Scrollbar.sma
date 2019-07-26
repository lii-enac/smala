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
import inverse_transform

// TODO: fix cairo bug with enter/leave when moving shape
// TODO: make it actually usable

_define_
Scrollbar(Process f) {
  // ---------------
  // model
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
  // transform
  Component transform {
    Double tx (200)
    Double ty (100)
    Double  s (100)
    Double rot (10)

    // impl
    Double sina (0)
    Double cosa (0)
    cos(0.01745329251 * transform.rot) =:> cosa
    sin(0.01745329251 * transform.rot) =:> sina
  }

  // graphical transform
  Rotation    rot(0,0,0)
  Translation tr(0,0)
  Scaling     sc(1,1, 0,0)
  
  transform.rot =:> rot.a
  transform.tx  =:> tr.tx
  transform.ty  =:> tr.ty
  transform.s   =:> sc.sy

  // -----------------
  // display view
  Double width (100)
  Double arrow_height(100)

  NoOutline _

  Component display_view {
    
    FillColor   _ (255,255,255) // white
    Rectangle   more_arrow (0,0,1,1,0,0) // ^
    FillColor   _ (200,200,200) // gray
    Rectangle   bg (0,0,1,1,0,0)        // []
    FillColor   _ (255,255,255) // white
    Rectangle   less_arrow (0,0,1,1,0,0) // v

    // thumb on top
    FillColor   _ (150,150,255) // grayblue
    Rectangle   thumb (0,0,1,1,0,0)      // =

    // 'one-way constraint' or data-flow of position/size of each zone for a regular scrollbar display layout
    // transformation of model into display view for background and thumb

                                         width =:> more_arrow.width, bg.width, thumb.width, less_arrow.width
                                             0 =:> more_arrow.x, bg.x, thumb.x, less_arrow.x

                                             0 =:> more_arrow.y 
                    arrow_height / transform.s =:> more_arrow.height
              more_arrow.y + more_arrow.height =:> bg.y
                                             1 =:> bg.height
                              bg.y + bg.height =:> less_arrow.y
                    arrow_height / transform.s =:> less_arrow.height

                         (1-model.high) + bg.y =:> thumb.y
                                   model.delta =:> thumb.height

  }

  // -----------------
  // picking view
  Int xoffset(300)

  // a picking view has a state that depends on the status of the interaction (see controller)
  Switch picking_view (initial) {

    Component initial {

      FillColor _ (0,255,0)//green
      Rectangle more_arrow (0,0,1,1,0,0)  // ^
      FillColor _ (0,255,255)//cyan
      Rectangle more_bg (0,0,1,1,0,0)     // ||
      FillColor _ (255,255,0)//yellow
      Rectangle less_bg (0,0,1,1,0,0)     // ||
      FillColor _ (255,0,0)//red
      Rectangle less_arrow (0,0,1,1,0,0)  // v

      // thumb on top
      FillColor _ (255,0,255)//purple
      Rectangle thumb (0,0,1,1,0,0)       // =
     
      // 'one-way constraint' or data-flow of position/size of each zone for a regular scrollbar picking layout
      // transformation of model into picking view for background and thumb     

                                         width =:> more_arrow.width, more_bg.width, thumb.width, less_bg.width, less_arrow.width
                                   0 + xoffset =:> more_arrow.x,     more_bg.x,     thumb.x,     less_bg.x,     less_arrow.x

                                             0 =:> more_arrow.y    // ^
                    arrow_height / transform.s =:> more_arrow.height
              more_arrow.y + more_arrow.height =:> more_bg.y       // ||
                              (1 - model.high) =:> more_bg.height  // ||
                 more_bg.y +    more_bg.height =:> thumb.y         // =
                                   model.delta =:> thumb.height    // =
                   thumb.y +      thumb.height =:> less_bg.y       // ||
                                     model.low =:> less_bg.height  // ||
                 less_bg.y +    less_bg.height =:> less_arrow.y    // v 
                    arrow_height / transform.s =:> less_arrow.height             
            
    }

    Component hyst {
      // the hysteresis circle is in screen space, so we should inverse graphical transform
      // should be LoadIdentity() it would be more efficient and onpar with the semantics
      Scaling     sc(1,1, 0,0)
      Translation tr(0,0)
      Rotation    rot(0,0,0)
      
       -transform.rot =:> rot.a
       -transform.tx  =:> tr.tx
       -transform.ty  =:> tr.ty
      1/transform.s   =:> sc.sy

      FillColor fc (255, 0, 0)
      Circle    c (0,0, 5)                   // °
    }

    Component dragging {
    
      FillOpacity _(0.5)
      FillColor _ (0,255,0) // g
      Rectangle upper_limit (0,0,1,1,0,0)    // || (!)
      FillColor _ (0,255,255) // c
      Rectangle dragging_zone (0,0,1,1,0,0)  // || (=)
      FillColor _ (255,0,0) // r
      Rectangle lower_limit (0,0,1,1,0,0)    // || (!)

      Double pick_offset (0)
      Double zero_in_model (0)   // does not work very well
      Double height_in_model (0) // idem
      
      // pick_offset in model coordinates
      inverse_transform _(transform, picking_view.hyst.c.cx, picking_view.hyst.c.cy, pick_offset)
      Double zero(0)
      inverse_transform _(transform, zero, zero, zero_in_model) // not working as planned
      inverse_transform _(transform, f.height, f.height, height_in_model) // not working as planned
     
      // 'one-way constraint' or data-flow of position/size for a regular scrollbar picking layout
                                         width =:> upper_limit.width, dragging_zone.width, lower_limit.width
                                       xoffset =:> upper_limit.x, dragging_zone.x, lower_limit.x

                                 zero_in_model =:> upper_limit.y

           picking_view.initial.more_bg.y
        + (pick_offset - picking_view.initial.thumb.y)
        -  upper_limit.y                       =:> upper_limit.height

          upper_limit.y +   upper_limit.height =:> dragging_zone.y
         1 - picking_view.initial.thumb.height =:> dragging_zone.height
        dragging_zone.y + dragging_zone.height =:> lower_limit.y
	             height_in_model - lower_limit.y =:> lower_limit.height
    } 
  }

  // -----------------
  // controller = management of interactive state with an FSM

  DoubleProperty lastv (0)

  FSM fsm {

    State idle {
      "initial" =: picking_view.state
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

    State paging_still // FIXME, enter is not working with cairo when moving shapes

    State waiting_hyst {
      "hyst" =: picking_view.state

      f.press.x =: picking_view.hyst.c.cx
      f.press.y =: picking_view.hyst.c.cy

      inverse_transform iv(transform, f.press.x, f.press.y, lastv)
    }

    State dragging {
      "dragging" =: picking_view.state
      
      // inverse transform from user actions to model operations
      Double dv (0)
      Double v (0)
      
      AssignmentSequence as(0) {
        // compute inverse and delta
        inverse_transform iv(transform, f.move.x, f.move.y, v)
        - (v - lastv) =: dv
        // remember last pos
        v =: lastv
        // apply to model
        dv + model.low  =: model.low
        dv + model.high =: model.high
      }
      f.move -> as
      
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
}


     /*
      Rectangle r(0,0,0,0,0,0)
      r.press -> fc 
      m aka r.inverted_matrix
      //Homography _(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1)
      Homography h
      m.m11 =: h.m11
      m.m12 =: h.m12
      m.m13 =: h.m13
      m.m14 =: h.m14
      m.m21 =: h.m21
      m.m22 =: h.m22
      m.m23 =: h.m23
      m.m24 =: h.m24
      m.m31 =: h.m31
      m.m32 =: h.m32
      m.m33 =: h.m33
      m.m34 =: h.m34
      m.m41 =: h.m41
      m.m42 =: h.m42
      m.m43 =: h.m43
      m.m44 =: h.m44

      //m.m11 =:> tp.input
      */
