use core
use base
use display
use gui

_define_
PanAndZoom (Process move, Process press, Process release, Process dw) {
    // "input"
    // move: e.g., frame.move // unfortunately, we need to know where the cursor is while zooming
    // press: e.g., frame.press
    // release: e.g., frame.release
    // dw: e.g., frame.wheel.dy, amout of delta zoom as input by the user

    // "output"
    Double zoom(1)
    Double xpan(0)
    Double ypan(0)


    // zoom management
    Double adw (0) // absolute dw
    Double tradw (0) // transfert function of adw
    Double dzoom (1) // resulting delta zoom

    fabs($dw) =:> adw 
    (0.00303 * adw + 0.00482) * adw + 1.001 =:> tradw
    // Pow p (1.01, 0)
    // adw =:> p.exponent
    // p.result =:> tradw
    dw >= 0 ? tradw : 1/tradw =:> dzoom

    // we need to know where the cursor is // FIXME put it in wheel event?..
    mouseTracking = 1

    Double new_zoom (1)
    AssignmentSequence zseq (1) {
        dzoom * zoom =: new_zoom
        xpan + move.x / new_zoom - move.x / zoom =: xpan
        ypan + move.y / new_zoom - move.y / zoom =: ypan
        new_zoom =: zoom
    }
    dzoom -> zseq

    // pan management

    Double xlast (0)
    Double ylast (0)

    FSM pan_control {
        State idle
        State pressing {
            move.x =: xlast
            move.y =: ylast
        }
        State panning {
            Double dx(0)
            Double dy(0)
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
