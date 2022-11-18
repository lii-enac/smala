use core
use gui
use base

import gui.shape.text

_action_
fn_init(Process src, Process data)
{
  int y = 12
  for t : data.choices {
    addChildrenTo data.fsm.unfolded.items {
      Component c {
        NoOutline _
        FillColor fc (#969696)
        Rectangle bkg (0, y - 14, 0, 18, 0, 0)
        data.box.width - 3 =:> bkg.width
        Spike select
        Spike unselect
        data.l_ask~>select
        data.l_ask~>unselect
        select~>data.fold
        unselect~>data.fold
        FillColor text_color (#000000)
        data.text_color.value =: text_color.value
        Text text (9, y, toString(t))
        width aka text.width
        FSM fsm {
          State idle {
            data.box_color.value =: fc.value
          }
          State hover {
            #FA9128 =: fc.value
          }
          idle->hover (bkg.enter)
          hover->idle (bkg.leave)
        }
        Switch sw (idle)  {
          Component idle {
            bkg.press -> { c =: data.ask_selection }
          }
          Component selected {
            FillColor color (#000000)
            data.text_color.value =: color.value
            Circle c (5, y - 4, 2)
            t =: data.choice.text
            bkg.press ->  { data.unselect_all =: data.ask_selection
                            "" =: data.choice.text
                          }
          }
        }
        FSM sel_fsm {
          State idle
          State selected
          idle->selected (select)
          selected->idle (unselect)
        }
        sel_fsm.state=:>sw.state, sel_fsm.initial
      }
    }
    y = y + 18
  }
}

_action_
fn_l_ask (Process src, Process data)
{
  p = getRef (&data.ask_selection)
  for c : data.fsm.unfolded.items {
    if (&c != &p) {
      notify c.unselect
    }
    if (&p != 0) {
      notify p.select
    }
  }
  notify data.fold
}

_define_
DropDownMenu (double _x, double _y)
{
  Translation pos (_x, _y)
  x aka pos.tx
  y aka pos.ty
  FontFamily ff("B612")
  OutlineColor border_color (#000000)
  FillColor box_color (50, 50, 50)

  Rectangle box (0, 0, 100, 18, 2, 2)
  FillColor text_color (#FFFFFF)
  Text choice (3, 14, "")
  selected aka choice.text

  RefProperty ask_selection (null)
  RefProperty unselect_all (null)

  FillColor button_color (40, 00, 250)
  Rectangle button (0, 0, 20, 18, 2, 2)
  box.width - 2 =:> button.x
  
  NativeAction l_ask (fn_l_ask, this, 1)
  List choices

  Spike fold
  
  FSM fsm {
    State idle {
      Translation pos (0, 0)
      OutlineColor _ (#FFFFFF)
      OutlineWidth _ (2)
      button.x =:> pos.tx
      button.y =:> pos.ty
      Polyline _ {
        Point _ (5, 7)
        Point _ (10, 13)
        Point _ (15, 7)
      }
    }
    State unfolded {
      Component _ {
        Translation pos (0, 0)
        OutlineColor _ (#FFFFFF)
        OutlineWidth _ (2)
        button.x =:> pos.tx
        button.y =:> pos.ty
        Polyline _ {
          Point _ (5, 12)
          Point _ (10, 6)
          Point _ (15, 12)
        }
      }
      FillColor fc (#000000)
      box_color.value =:> fc.value
      OutlineColor _ (#000000)
      Rectangle rec_list (0, -2, 100, 0, 0, 0)
      choices.size * 18  + 4 =:> rec_list.height
      box.width - 2 =:> rec_list.width
      Translation _ (1, 2)
      List items
    }
    idle->unfolded (button.press)
    unfolded->idle (button.press)
    unfolded->idle (fold)
  }
  
  
  MaxList sum (fsm.unfolded.items, "width")
  NativeAction init (fn_init, this, 0)
  sum.output + 20 =:> box.width

  ask_selection->l_ask
  l_ask ~> selected

  
}