use core
use base
use display
use gui

_native_code_
%{
#include <math.h>
%}

_define_
PanAndZoom (Process move, Process press, Process release, Process dwheel) {
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


    // zoom management

    // transfer function
    Double adw (0) // absolute dw
    Double transf_adw (0) // transfert function of adw    

    fabs($dwheel) =:> adw 
    (0.00303 * adw + 0.00482) * adw + 1.001 =:> transf_adw // polynomial-based transfer function
    // Pow p (1.01, 0) // exponent-based transfer function
    // adw =:> p.exponent
    // p.result =:> transf_adw

    // inversing zoom according to dwheel sign
    dwheel >= 0 ? transf_adw : 1/transf_adw =:> dzoom

    // we need to know where the cursor is // FIXME put it in wheel event?..
    mouseTracking = 1
    Double last_move_x (0)
    Double last_move_y (0)
    move.x =:> last_move_x
    move.y =:> last_move_y

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

    BoundedValue bv (0.20, 2.30, 1)
    AssignmentSequence pre_zseq (1) {
        dzoom * zoom =: bv.input
    }
    dzoom -> pre_zseq

    AssignmentSequence zseq (1) {
        bv.result =: new_zoom
        last_move_x / new_zoom - last_move_x / zoom =: dx
        last_move_y / new_zoom - last_move_y / zoom =: dy
        xpan + dx =: new_xpan
        ypan + dy =: new_ypan

        new_zoom =: zoom
        new_xpan =: xpan
        new_ypan =: ypan
    }
    bv.result -> zseq


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
                xpan + dx =: new_xpan
                ypan + dy =: new_ypan
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
