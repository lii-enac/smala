use core
use base
use display
use gui

_native_code_
%{
#include <math.h>

inline
double myfract(double f)
{
  return f - floor(f);
}

inline
int myfloor(double f)
{
  return floor(f);
}


%}

_define_
PanAndZoomGrid (Process move, Process press, Process release, Process wheel) {
    // input

    // move: e.g., frame.move // unfortunately, we need to know where the cursor is while zooming // FIXME put it in wheel event?..
    // press: e.g., frame.press
    // release: e.g., frame.release
    // dwheel: e.g., frame.wheel.dy, amoumt of delta zoom input by the user


    // output // should be connected e.g. to a Translation and a Scaling

    Double zoom (1)
    Double xpan (0)
    Double ypan (0)


    // delta output, in case it's useful

    Double dzoom (1)
    Double dx (0)
    Double dy (0)

    // grid zoom level
    Double actual_zoom ($zoom)
    Int zoom_level(1)

    Double actual_xpan ($xpan)
    Double actual_ypan ($ypan)
    
    // zoom management

    // transfer function to compute dzoom (i.e. new_zoom / old_zoom)
    Double abs_dw (0) // absolute dw
    Double transf_abs_dw (0) // transfert function of adw    

    fabs($wheel.dy) =:> abs_dw 
    (0.00303 * abs_dw + 0.00482) * abs_dw + 1.001 =:> transf_abs_dw // polynomial-based transfer function
    // Pow p (1.01, 0) // exponent-based transfer function
    // abs_dw =:> p.exponent
    // p.result =:> transf_abs_dw

    // inversing zoom according to wheel movement sign
    wheel.dy >= 0 ? transf_abs_dw : 1/transf_abs_dw =:> dzoom


    // we need to know where the cursor is
    Double last_move_x (0)
    Double last_move_y (0)
    wheel.x =:> last_move_x
    wheel.y =:> last_move_y


    Double new_zoom (1)
    Double new_xpan (0)
    Double new_ypan (0)

    // --- it could have been the following:
    // dzoom * zoom =:> new_zoom
    // xpan + move.x / new_zoom - move.x / zoom =:> new_xpan
    // ypan + move.y / new_zoom - move.y / zoom =:> new_ypan

    // --- since we update zoom in new_zoom, we must avoid creating a coupling between them, as that would build a cycle
    // this should be harmless, however the activation vector is not cleaned after exec anymore for performance reasons, so it is harmful!
    // so the above connectors would theoritically be alright, but not practical as compared to assignments, since they create a coupling
    // hence we move them as assignments into an AssignmentSequence instead:

    Double zoom_level_d(1.0)
    Double zld_01(0.0)
    //Double local_zoom(1.0)

    AssignmentSequence zseq (1) {
        dzoom * actual_zoom =: actual_zoom

        dzoom * zoom =: new_zoom

        // compute the 2 exponent
        log2($new_zoom) =: zoom_level_d
        // then constrain its value to [0,1]
        myfract($zoom_level_d) =: zld_01
        // apply it to set our new_zoom
        pow (2, $zld_01) =: new_zoom
        // recompute dzoom according to new_zoom
        new_zoom / zoom =: dzoom
        // that's it, pan should follow suit

        // dxy is the diff between last_move_x after new_zooom and before new_zoom
        last_move_x / new_zoom  -  last_move_x / zoom =: dx
        last_move_y / new_zoom  -  last_move_y / zoom =: dy
        xpan + dx =: new_xpan
        ypan + dy =: new_ypan

        actual_xpan + dx =: actual_xpan
        actual_ypan + dy =: actual_ypan

        //new_xpan % 256 =: new_xpan
        //new_ypan % 256 =: new_ypan

        new_zoom =: zoom
        new_xpan =: xpan
        new_ypan =: ypan
    }
    dzoom -> zseq

    TextPrinter tp

    Int tmp_zl(1.0)
    myfloor(1 + log2($actual_zoom)) =:> tmp_zl
    tmp_zl =?> zoom_level
    //zoom_level =:> tp.input

    Int xgrid(0)
    Int tmp_xgrid(0)
    myfloor ( (actual_xpan) / 256.) =:> tmp_xgrid
    tmp_xgrid =?> xgrid
    
    Int ygrid(0)
    Int tmp_ygrid(0)
    myfloor ( (actual_ypan) / 256.) =:> tmp_ygrid
    tmp_ygrid =?> ygrid

    Spike left
    Spike right
    Spike up
    Spike down

    

    //"xgrid " + toString(xgrid) =:> tp.input
    //"ygrid " + toString(ygrid) =:> tp.input
    
    
    //xpan =:> tp.input

    // pan management

    Double xlast (0)
    Double ylast (0)

    FSM pan_control {
        State idle
        State pressing {
            press.x =: xlast
            press.y =: ylast
        }
        State panning {
            AssignmentSequence seq (1) {
                (move.x - xlast) / zoom =: dx
                (move.y - ylast) / zoom =: dy
                //(move.x - xlast) =: dx
                //(move.y - ylast) =: dy
                xpan + dx =: new_xpan
                ypan + dy =: new_ypan
                actual_xpan + dx =: actual_xpan
                actual_ypan + dy =: actual_ypan
                //new_xpan % 256 =: new_xpan
                //new_ypan % 256 =: new_ypan
                new_xpan =: xpan
                new_ypan =: ypan

                move.x =: xlast
                move.y =: ylast
            }
            move -> seq

        }
        idle -> pressing (press)
        pressing -> idle (release)
        pressing -> panning (move)
        panning -> idle (release)
    }

}
