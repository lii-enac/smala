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
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

 use core
 use base
 use display
 use gui

 import gui.widgets.StandAlonePushButton

 _action_
 list_action (list l, Process c)
 %{
  //Process *data = (Process*) get_native_user_data (c);
  for (auto e: l) {
    double w = ((AbstractProperty*)e->find_child("width"))->get_double_value ();
    ((DoubleProperty*)e->find_child("width"))->set_value (w + 5, true);
  }
  %}


  _main_
  Component root {

    Frame f ("f", 0, 0, 500, 600)
    Exit ex (0, 1)
    f.close -> ex
    NoOutline _
    FillColor _ (10, 10, 10)
    Rectangle bkg (0, 0, 0, 0, 0, 0)
    f.width =:> bkg.width
    f.height =:> bkg.height
    FillColor fcc (#FFFFFF)
    Text explanation1 (10, 20, "Select/unselect rectangles by clicking on them")
    Text explanation2 (10, 40, "Click on the Increase/Decrease buttons to increase/decrease their size")
    Text explanation3 (10, 60, "Unselect all by clicking on the Clear button")

    Int select (#28F0FA)
    Int unselect (#0A0A0A)

    OutlineWidth _ (4)
    Spike rm_all
    ProcessCollector collection
    
    List rectangles {
      Component _ {
        FillColor _ (#FF0000)
        OutlineColor oc ($unselect)
        Rectangle rec (50, 400, 100, 100, 0, 0)
      }
      Component _ {
        FillColor _ (#00FF00)
        OutlineColor oc ($unselect)
        Rectangle rec (200, 400, 100, 100, 0, 0)
      }
      Component _ {
        FillColor _ (#0000FF)
        OutlineColor oc ($unselect)
        Rectangle rec (350, 400, 100, 100, 0, 0)
        
      }
    }
    StandAlonePushButton inc ("Increase", 50, 150)
    StandAlonePushButton dec ("Decrease", 150, 150)
    StandAlonePushButton clear ("Clear", 250, 150)

    for r : rectangles {
      addChildrenTo r {
        FSM fsm {
          State unselected {
            unselect =: r.oc.value
            r.rec =: collection.rm
          }
          State selected {
            select =: r.oc.value
            r.rec =: collection.add
          }
          unselected->selected (r.rec.press)
          selected->unselected (r.rec.press)
          selected->unselected (rm_all)
        }
      }
    }

    clear.click->collection.rm_all, rm_all

    NativeCollectionAction coll_act (list_action, collection, 1)
    inc.click->coll_act
    dec.click->(collection) {
      for r : collection {
        r.width = r.width - 5
      }
    }
  }

