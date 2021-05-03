/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020-2021)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */
use core

_native_code_
%{
  #include <iostream>
  using namespace std;
%}


_define_
ShareCursorPosition (Process frame, Process ivybus, string uid_p, int color_p, int period_p)
{
    String uid (uid_p)
    Int color (color_p)
    Double _to_send_x (0)
    Double _to_send_y (0)
    Bool _has_moved (0)
    //TextPrinter tp

    // -------------- Option 1 -------------- //
    // Every move, worse for performances (spam on network)
    /*frame.move -> {
        "CursorMoved Uid=" + uid + " Color=" + color + " X=" + frame.move.x + " Y=" + frame.move.y =: ivybus.out
    }*/


    // -------------- Option 2 -------------- //
    // Less precise, better for performances
    Switch switch (false) {
        Component true {
            //"CursorMoved Uid=" + uid + " Color=" + color + " X=" + _to_send_x + " Y=" + _to_send_y =:> tp.input
            "CursorMoved Uid=" + uid + " Color=" + color + " X=" + _to_send_x + " Y=" + _to_send_y =:> ivybus.out
        }
        Component false {
            //"CursorPaused Uid=" + uid =: tp.input
            "CursorPaused Uid=" + uid =: ivybus.out
        }
    }
    _has_moved =:> switch.state

    Clock clock (period_p)
    clock.tick -> {
        (_to_send_x != frame.move.x) || (_to_send_y != frame.move.y) =: _has_moved
        
        // Will NOT be sent if _has_moved is false
        frame.move.x =: _to_send_x 
        frame.move.y =: _to_send_y
    }
}