/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2025)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Poirier <mathieu.poirier@enac.fr>
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

// using namespace h2d;

// shortcut to "get double value"
#define GDV(proc, prop) proc->prop()->get_double_value()
// shortcut to "set double value"
#define SDV(proc, prop, v) proc->prop()->set_value((v), true)


void
cpp_action(Process *p)
{
    Process *root = (Process*) get_native_user_data (p);
    GET_CHILD_VAR (l, Line, root, l);
    GET_CHILD_VAR (cinter, Circle, root, cinter);
    GET_CHILD_VAR (c1, Circle, root, intercept/c1);
    GET_CHILD_VAR (c2, Circle, root, intercept/c2);

    try {

        using namespace h2d;

        Segment l_(
            Point2d(GDV(l, x1), GDV(l, y1)),
            Point2d(GDV(l, x2), GDV(l, y2))
        );
        
        Point2d center = Point2d(GDV(cinter, cx), GDV(cinter, cy));
        h2d::Circle circle_ = h2d::Circle (center, GDV(cinter, r));

        auto res = l_.intersects(circle_); // get () will return a vector : https://github.com/skramm/homog2d/blob/master/docs/homog2d_manual.md

        if(res()){
            if (res.get().size () >= 1) {
                auto pt = res.get()[0];
                SDV(c1, cx,pt.getX());
                SDV(c1, cy,pt.getY());
                SDV(c1, r,5);
            }else{
                SDV(c1, r,0);
            }

            if (res.get().size () >= 2) {
                auto pt = res.get()[1];
                SDV(c2, cx, pt.getX());
                SDV(c2, cy, pt.getY());
                SDV(c2, r,5);
            }else{
                SDV(c2, r,0);
            }
        } else {
            SDV(c1, r,0);
            SDV(c2, r,0);
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
    Circle cinter(0,0,50)

    Group intercept {
        FillColor _ (255,0,0)
        Circle c1 (0,0,5)
        Circle c2 (0,0,5)
    }

    Spike resizing
    FSM fsm {
        State idle {
            -100 =:  cinter.cx, cinter.cy // hide the circle
            0 =:  intercept.c1.r // hide intersection
            0 =:  intercept.c2.r // hide intersection
        }
        State start {
            f.press.x =: cinter.cx //, linter.x2
            f.press.y =: cinter.cy //, linter.y2
            5 =: intercept.c1.r
            5 =: intercept.c2.r
        }
        State line_resize {
            f.move.x => cinter.cx
            f.move.y => cinter.cy
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