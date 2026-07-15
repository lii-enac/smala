/*
*  Smala Library
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
import display.window

import gui.widgets.IWidget


_native_code_
%{
#include "core/utils/iostream.h"
#include "base/process_handler.h"

//using string = djnnstl::string;

static
Process*
check (Process* p, const djnnstl::string& p_name)
{
  return find_optional (p, p_name);
}

inline
void myprint(const djnnstl::string& s)
{
  puts(s.c_str());
}


template <typename PROCESS>
static
PROCESS*
find_process (CoreProcess* obj)
{
    /*  this algorithm is a little bit tricky. We want to find the closest running frame
    *  on the left side of the current object (cur_child). For this, we take its parent (cur_parent ()) and go through its
    *  children in order to find a frame. If no frame is found when the list iteration process arrived to (cur_child),
    *  then we set (cur_child) to its parent (cur_parent ()), and (cur_parent ()) is set to (cur_parent ()->parent).
    *  May be there is a place for simplification */
    bool         found      = false;
    FatProcess*  cur_parent = obj->get_parent ();
    CoreProcess* cur_child  = obj;

    PROCESS* to_find = nullptr;

    while (!found && cur_parent != nullptr) {
        if (cur_parent->get_process_type () == CONTAINER_T) {
            Container* cont = dynamic_cast<Container*> (cur_parent);
            for (auto c : cont->children ()) {
                //std::cerr << c->get_debug_name() << std::endl;
                if (c == cur_child)
                    break;
                else /*if (c->somehow_activating ())*/ {
                    to_find = dynamic_cast<PROCESS*> (c);
                    //std::cerr << to_find << std::endl;
                    if (to_find) {
                        found = true;
                        break; // FIXME find_frame
                    }
                }
            }
        }

        if (!found) do { // FIXME find_frame
            cur_child  = cur_parent;
            cur_parent = cur_parent->get_parent ();
        } while (cur_parent != nullptr && cur_parent->get_process_type () != CONTAINER_T);
    }

    if (!found) {
        //puts("not found");
        return nullptr;
    }

    return to_find;
}

static
Window*
my_find_frame(CoreProcess* obj)
{
    return find_process<Window>(obj);
}

static
void
add_if_not_yet(CoreProcess* pc_, CoreProcess* child)
{
  auto pc = dynamic_cast<ProcessCollector*>(pc_);
  //assert (pc);
  if (pc) {
    bool exist = false;
    for (CoreProcess* widget : pc->get_list ()) {
      if (widget == child) {
        exist = true;
        break;
      }
    }
    if (!exist) {
      pc->add_one(child);
    }
  }
}

%}

_define_
AbstractBox () inherits IWidget ()
{
  parent = find(this, "..")
  Process container = null
  Int top_level_box(0)
  if (find_optional(parent, "width")==0) {
    frame = my_find_frame(this)
    container = &frame
    top_level_box = 1
  } else {
    container = &parent
  }

  check (container, "width")
  check (container, "height")

  Bool set_pos (1)
  Translation offset (0, 0)
  off_x aka offset.tx
  off_y aka offset.ty
  Translation padding (5, 5)
  padding_left aka padding.tx
  padding_top aka padding.ty
  Int cell_width (0)
  Int cell_height (0)
  Int space (5)

  if (find_optional (container, "cell_width")) {
    //print ("yes")
    container_width aka container.cell_width
    container_height aka container.cell_height
    set_pos = 0
    padding_left = 0
    padding_top = 0
  } else {
    //print ("no")
    container_width aka container.width
    container_height aka container.height
    //print(container_width)
  }

  ProcessCollector items(0)

  // TextPrinter tp
  // items.size + " items in box" => tp.input

  Component remaining // will be moved at the end, serves as a marker for additional components

  this.pack -> (this) {
    // print("\n******")
    // string s_this = get_debug_name(this)
    // myprint(s_this)
    int found_remaining = 0
    for item : this {
      //string n = get_debug_name(item)
      //myprint(n)

      if (&item == &this.remaining) {
        // print ("...remaining")
        found_remaining = 1
        continue
      }
      if (found_remaining > 0) {
          // string n = get_debug_name(item)
          // myprint(n)
          if (find_optional (item, "pack")) {
            //print("--- pack")
            //notify item.pack // FIXME rename to propagate?
            activate (item.pack)
            
            add_if_not_yet (this.items, item)
          }
          // else {
          //   string n = get_debug_name(item)
          //   print ("warning:")
          //   myprint (n)
          //   print (" is not a IWidget")
          // }
      }
    }
    // print("END\n")
  }

//   NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
//   this.container_width -> update_items_pos_and_geom
//   this.container_height -> update_items_pos_and_geom

//   if (this.top_level_box == 1) {
//     Timer t(1)
//     t.end -> this.pack     // FIXME
//     t.end -> update_items_pos_and_geom
//     this.pack ~> update_items_pos_and_geom
//   }

/*
  NativeAction update_items_pos_and_geom (fn_update_items_pos_and_geom, this, 0)
  this.container_width -> update_items_pos_and_geom
  this.container_height -> update_items_pos_and_geom
  SumList sl (items, "min_width")
  MaxList ml (items, "min_height")
  sl.output -> update_items_pos_and_geom // bad trick to force geometry recomputation at startup, needed for text
  sl.output + space*(items.size - 1) =:> this.min_width
  ml.output =:> this.min_height

  if (top_level_box == 1) {
    Timer t(1)
    t.end -> pack     // FIXME
    t.end -> update_items_pos_and_geom
    pack ~> update_items_pos_and_geom
  }
  
  moveChild remaining >>
  */
}