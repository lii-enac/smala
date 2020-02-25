/*
 *  dock app
 *
 *  The copyright holders for the contents of this file are:
 *  Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *  Contributors:
 *     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

import Icon

_main_
Component root
{
  Frame f ("Dock", 0, 0, 512, 512)
  mouseTracking = 1

  FillColor _ (30, 30, 30)
  NoOutline _ ()
  Rectangle recf (0, 0, 0, 0, 0, 0)

  f.width =:> recf.width
  f.height =:> recf.height

  Component menu {
    svg = loadFromXML ("menu.svg")
    Translation pos_menu (-140, 0)
    full << svg.layer1
    add aka full.add
    del aka full.del
    tip aka full.tip
    Spike end_anim ()
    Incr inc (1)
    1 =: inc.state
    -140 * inc.state =:> pos_menu.tx
    FSM fsm {
      State folded {
        1 =: inc.state
      }
      State unfold {
        Clock cl (20)
        -0.1 =: inc.delta
        cl.tick->inc
        inc.state <= 0 -> end_anim
      }
      State unfolded {
        0 =: inc.state
      }
      State fold {
        Clock cl (20)
        0.1 =: inc.delta
        cl.tick->inc
        inc.state >= 1 -> end_anim
      }
      folded->unfold (tip.enter)
      unfold->unfolded (end_anim)
      unfolded->fold (recf.enter)
      fold->folded (end_anim)
    }
  }



  Translation pos (0, 0)
  Component bkg {
    FillOpacity _ (0.6)
    FillColor _ (White)
    Rectangle r (0, 0, 50, 70, 5, 5)
  }

  f.height - 65 =:> pos.ty
  f.width/2 - bkg.r.width/2 =:> pos.tx
  FillColor _ (Pink)


  Ref to_delete (null)
  Spike update_size
  Spike del
  Spike add
  menu.del.press->del
  menu.add.press->add
  List icons {
  }

  SumList sum (icons, "width")
  sum.output + 10 => bkg.r.width

  menu.add.press->(root) {
    addChildrenTo root.icons {
      int sz = $root.icons.size
      if (sz > 0) {
        Icon _ (root, root.icons.[sz], 0, 10)
      }
      else {
        Icon _ (root, null, 0, 10)
      }
    }
  }
  update_size->(icons) {
    for (int i = 2; i <= $icons.size; i++) {
      int prev = i - 1
      setRef (&icons.[i].prev, &icons.[prev])
    }
  }
  to_delete->(root) {
    item = getRef (&root.to_delete)
    if (&item != null) {
      for (int i = 2; i <= $root.icons.size; i++) {
        delete root.icons.[i].pos_cnt
      }
      delete item
      setRef (&root.to_delete, null)
      run root.update_size
    }
  }
}

run root
run syshook
