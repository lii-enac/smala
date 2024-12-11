/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Stéphane Conversy <stephane.conversy@enac.fr>
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

void
cpp_action(Process *p)
{
    Process *root = (Process*) get_native_user_data (p);
    GET_CHILD_VAR (l, Line, root, l);
    GET_CHILD_VAR (linter, Line, root, linter);
    GET_CHILD_VAR (c, Circle, root, intercept/c);

    using namespace h2d;
    Line2d l_(
        Point2d(l->x1()->get_double_value(), l->y1()->get_double_value()),
        Point2d(l->x2()->get_double_value(), l->y2()->get_double_value())
        );
    Line2d linter_(
        Point2d(linter->x1()->get_double_value(), linter->y1()->get_double_value()),
        Point2d(linter->x2()->get_double_value(), linter->y2()->get_double_value())
        );
	
    try {
        auto pt = l_ * linter_;
        c->cx()->set_value (pt.getX(), true);
        c->cy()->set_value (pt.getY(), true);
    } catch (const std::exception& err) {

    }
}
%}

_main_
Component root
{
    Frame f ("my frame", 0, 0, 1280, 720)
    Exit ex (0, 1)
    f.close -> ex
    //mouseTracking = 1

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
            0 =: linter.x1
            0 =: linter.y1
            0 =: linter.x2
            0 =: linter.y2
        }
        State start {
            f.press.x =: linter.x1, linter.x2
            f.press.y =: linter.y1, linter.y2
        }
        State line_resize {
            f.move.x =:> linter.x2
            f.move.y =:> linter.y2
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