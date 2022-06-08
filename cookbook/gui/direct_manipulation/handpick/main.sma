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
 *    Mathieu Poirier <mathieu.poirier@enac.fr>
 *
 */

use core
use base
use display
use gui

_native_code_
%{
#include <iostream>
#include <cassert>
#include "gui/picking/picking.h"

void
pick(Process *na)
{
  auto *p = na->get_parent();
  auto * circle = dynamic_cast<Circle*>(p->find_child("c"));
  assert(circle);
  auto * poly = dynamic_cast<Poly*>(p->find_child("p/p"));
  assert(poly);
  auto * frame = dynamic_cast<Window*>(p->find_child("f"));
  assert(frame);
  auto * inside = dynamic_cast<Spike*>(p->find_child("inside"));
  assert(inside);
  auto * outside = dynamic_cast<Spike*>(p->find_child("outside"));
  assert(outside);

  Picking * picking = frame->picking_view();
  assert(picking);

  auto * picked = picking->pick(circle->cx()->get_double_value(), circle->cy()->get_double_value());

  if(picked) {
    inside->notify_activation();
  } else {
    outside->notify_activation();
  }
}

%}

_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close->ex
  
  // the polygon to test "insideness"
  Component p {
    FillColor fc(0,255,0)
    Translation _(-500, 0)
    Poly p(1) {
      Point _(867.,          256.        )
      Point _(945.98376465,  385.31274414)
      Point _(798.5927124,   350.15460205)
      Point _(700.01623535,  465.23242188)
      Point _(687.9072876,   314.19073486)
      Point _(548.,          256.        )
      Point _(687.9072876,   197.80926514)
      Point _(700.01623535,   46.76756668)
      Point _(798.5927124,   161.84539795)
      Point _(945.98376465,  126.68724823)    
    }

    Spike dummy
    p.press -> dummy // make p pickable, to make it appear in the picking view
  }

  // the shape that will be inside the polygon
  FillColor _(255,0,0)
  Circle c(100,100,10)

  // stuff to trigger picking and reaction to picking
  Spike inside
  Spike outside
  NativeAction na(pick, root, 1)
  c.cx -> na
  c.cy -> na

  // reaction to picking
  FSM fsm {
    State out {
      0 =: p.fc.r
    }
    State in {
      255 =: p.fc.r
    }
    out->in(inside)
    in->out(outside)
  }

  // animate the circle above the polygon
  Incr i(1)
  10 =: i.delta
  Clock cl(100)
  cl.tick -> i
  i.state =:> c.cx

}
