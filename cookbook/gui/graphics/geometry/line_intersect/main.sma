/*
*  Smala cookbook line_intersect
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Stéphane Conversy <stephane.conversy@enac.fr>
*
*/
use core
use base
use display
use gui

_native_code_
%{
#include "homog2d.h"
#include "core/utils/getset.h"

// shortcut to "get double value"
#define GDV(proc, prop) proc->prop()->get_double_value()
// shortcut to "set double value"
#define SDV(proc, prop, v) proc->prop()->set_value((v), true)

void
cpp_action(Process *p)
{
    Process *root = (Process*) get_native_user_data (p);
    GET_CHILD_VAR (l, Line, root, l);
    GET_CHILD_VAR (linter, Line, root, linter);
    GET_CHILD_VAR (c, Circle, root, intercept/c);

    try {
        using namespace h2d;
        Segment l_(
            h2d::Point2d(GDV(l, x1), GDV(l, y1)),
            h2d::Point2d(GDV(l, x2), GDV(l, y2))
            );
        Segment linter_(
            h2d::Point2d(GDV(linter, x1), GDV(linter, y1)),
            h2d::Point2d(GDV(linter, x2), GDV(linter, y2))
            );

        //auto pt = l_ * linter_;
        auto res = l_.intersects(linter_);
        if(res()){
            auto pt = res.get();
            SDV(c, cx,pt.getX());
            SDV(c, cy,pt.getY());
            SDV(c, r,5);
        }else{
            SDV(c, r,0);
        }
       
    } catch (const std::runtime_error& err) {
        // if lines are not well-formed or are parallel
    }
}
%}

_main_
Component root
{
    Frame f ("my frame", 0, 0, 1280, 720)
    Exit ex (0, 1)
    f.close -> ex

    OutlineColor _(255,255,0)
    Line l(0,0,500,500)
    Line linter(0,0,0,0)

    Group intercept {
        FillColor _(255,0,0)
        Circle c(0,0,5)
    }

    Spike resizing
    FSM fsm {
        State idle {
            0 =: linter.x1, linter.y1, linter.x2, linter.y2 // hide the line
            0 =:  intercept.c.r // hide intersection
        }
        State start {
            f.press.x =: linter.x1, linter.x2
            f.press.y =: linter.y1, linter.y2
            5 =: intercept.c.r
        }
        State line_resize {
            f.move.x => linter.x2
            f.move.y => linter.y2
            f.move -> resizing
        }
        idle -> start (f.press)
        start -> idle (f.release)
        start -> line_resize (f.move)
        line_resize -> idle (f.release)
    }

    NativeAction na(cpp_action, root, 1)
    resizing -> na
}