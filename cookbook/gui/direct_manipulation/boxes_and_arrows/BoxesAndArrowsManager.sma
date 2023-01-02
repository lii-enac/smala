use core
use base
use gui

import Arrow

_native_code_
%{
#include <map>
#include <iostream>

std::map<Process*, Process*> arrow_map;

void add_arrow (Process* src, Process *dst)
{
  arrow_map.insert(std::pair<Process*, Process*> (src, dst));
}

int arrow_exists (Process* src, Process *dst)
{
  std::map<Process*, Process*>::iterator it;
  it = arrow_map.find (src);
  if (it != arrow_map.end () && it->second == dst) {
    return 1;
  }
  return 0;
}
%}

_define_
BoxesAndArrowsManager (Process _gui)
{
  gui aka _gui
  RefProperty cur_box (null)
  RefProperty cur_arrow (null)

  Spike cancel
  cancel->(this) {
    arrow = getRef (this.cur_arrow)
    if (&arrow != null) {
      delete arrow
    }
  }

  FSM fsm_mode {
    State idle {
      cur_box->new_arrow:(this) {
        src = getRef (this.cur_box)
        if (&src != null) {
          addChildrenTo this.gui.arrows {
            Arrow a (src)
            setRef (this.cur_arrow, a)
          }
        } else {
            print ("null src")
        }
      }
    }
    State search_target {
      GenericMouse.left.release->end_arrow:(this){
        dst = getRef (this.cur_box)
        arrow = getRef (this.cur_arrow)
        arrow_src = getRef (arrow.src)
        if (arrow_exists (arrow_src, dst)) {
          notify this.cancel
          return
        }
        if (&dst != &arrow_src) {
          setRef (arrow.dst, dst)
          setRef (this.cur_arrow, null)
          add_arrow (arrow_src, dst)
        } else {
          notify this.cancel
        }
      }
    }
    idle->search_target(idle.new_arrow)
    search_target->idle (search_target.end_arrow)
    search_target->idle (cancel)
  }
}