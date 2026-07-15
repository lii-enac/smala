/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui



_define_
PixmapLayer (int reset_time, int pad, Process frame, Process scale, Process translate, Process rotation, Process canvas) {
    
    Spike do_damage

    Rotation outer_inv_rotation (0, 0, 0) // inverse
    Translation outer_inv_translate (0,0) // inverse
    Scaling outer_inv_scale (1,1, 0,0) // inverse
    if (&rotation) {
        rotation.{cx, cy} =:> outer_inv_rotation.{cx, cy}
    }

    Layer pixmap (pad) {

        //debug
        // FillColor _ (Yellow)
        // Rectangle debug_pad (0, 0, 0, 0, 0, 0)
        // FillColor _ (Green)
        // Rectangle debug (0, 0, 0, 0, 0, 0) 

        Scaling inner_scale (1, 1, 0, 0)
        Translation inner_translate (0, 0)
        Rotation inner_rotation (0, 0, 0)
        if (&rotation) {
            rotation.{cx, cy} =:> inner_rotation.{cx, cy}
        }

        frame.width =:> pixmap.width
        frame.height =:> pixmap.height
        //debug
        // frame.width =:> pixmap.debug.width  
        // frame.height =:> pixmap.debug.height
        // frame.width + 2 * pixmap.pad =:> pixmap.debug_pad.width
        // frame.height + 2 * pixmap.pad =:> pixmap.debug_pad.height
        // pixmap.x - pixmap.pad =:> pixmap.debug_pad.x
        // pixmap.y - pixmap.pad =:> pixmap.debug_pad.y 

        << canvas

        FSM damageLayerFSM {
            State idle {
                if (&rotation) {
                    -rotation.a =: outer_inv_rotation.a
                }
                if (&translate) {
                    -translate.tx =: outer_inv_translate.tx
                    -translate.ty =: outer_inv_translate.ty
                }
                if (&scale) {
                    1./scale.sx =: outer_inv_scale.sx
                    1./scale.sy =: outer_inv_scale.sy
                }
          
                if (&scale) {
                    scale.sx =: pixmap.inner_scale.sx
                    scale.sy =: pixmap.inner_scale.sy
                }
                if (&translate) {
                    translate.tx =: pixmap.inner_translate.tx
                    translate.ty =: pixmap.inner_translate.ty
                }
                if (&rotation) {
                    rotation.a =: pixmap.inner_rotation.a
                }
            }
            State waiting {
                Timer t (reset_time) // beware, make sure the time is greater than the time it takes to render a frame, otherwise the layer will constantly be damaged
                t.end -> pixmap.damaged
                do_damage -> t.reset
            }
            idle -> waiting (do_damage)
            waiting -> idle (waiting.t.end)
      }
    }
}
