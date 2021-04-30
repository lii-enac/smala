use core
use base
use display
use gui

_native_code_
%{
#include <math.h>
%}

_define_
PanAndZoom (Process move, Process press, Process release, Process dw) {
    // "input"
    // move: e.g., frame.move // unfortunately, we need to know where the cursor is while zooming
    // press: e.g., frame.press
    // release: e.g., frame.release
    // dw: e.g., frame.wheel.dy, amout of delta zoom as input by the user

    // "output"
    Double zoom (1)
    Double xpan (0)
    Double ypan (0)


    // zoom management
    Double adw (0) // absolute dw
    Double transf_adw (0) // transfert function of adw
    Double dzoom (1) // resulting delta zoom

    fabs($dw) =:> adw 
    (0.00303 * adw + 0.00482) * adw + 1.001 =:> transf_adw // polynomial-based transfer function
    // Pow p (1.01, 0) // exponent-based transfer function
    // adw =:> p.exponent
    // p.result =:> transf_adw
    dw >= 0 ? transf_adw : 1/transf_adw =:> dzoom

    // we need to know where the cursor is // FIXME put it in wheel event?..
    mouseTracking = 1
    Double last_move_x (0)
    Double last_move_y (0)
    move.x =:> last_move_x
    move.y =:> last_move_y

    Double new_zoom (1)
    Double new_xpan (0)
    Double new_ypan (0)

    // since we update zoom in new_zoom later, we must avoid creating a coupling between them, as that would build a cycle
    // This should be harmless, however the activation vector is not cleaned after exec anymore for performance reasons, so it is harmful!
    // so the following would theoritically be alright, but not practically: we put it in the AssignmentSequence instead
    // dzoom * zoom =:> new_zoom
    // xpan + move.x / new_zoom - move.x / zoom =:> new_xpan
    // ypan + move.y / new_zoom - move.y / zoom =:> new_ypan

    AssignmentSequence zseq (1) {
        dzoom * zoom =: new_zoom
        xpan + last_move_x / new_zoom - last_move_x / zoom =: new_xpan
        ypan + last_move_y / new_zoom - last_move_y / zoom =: new_ypan

        new_zoom =: zoom
        new_xpan =: xpan
        new_ypan =: ypan
    }
    dzoom -> zseq


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
            Double dx (0)
            Double dy (0)
            AssignmentSequence seq (1) {
                move.x - xlast =: dx
                move.y - ylast =: dy
                xpan + dx / zoom =: xpan
                ypan + dy / zoom =: ypan
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
