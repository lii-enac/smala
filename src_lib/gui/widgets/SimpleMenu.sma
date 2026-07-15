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
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use gui
use base
use gui

_action_
fn_init(Process src, Process data)
{
  int y = data.default_height
  for t : data.choices {
    addChildrenTo data.fsm.unfolded.items {
      Component c {
        NoOutline _
        FillColor fc (White)
        Rectangle bkg (2, y - 16, 0, 18, 0, 0)
        data.width - 3 =:> bkg.width
        FillColor text_color (#000000)
        data.text_color.value =: text_color.value
        Text text (5, y, toString(t))
        width aka text.width
        FSM fsm {
          State idle {
            data.bg_color.value =: fc.value
          }
          State hover {
            data.text_selection_color.value =: fc.value
          }
          idle->hover (bkg.enter)
          hover->idle (bkg.leave)
        }
        bkg.press ->  {
          text.text =: data.selected
        }
      }
    }
    y = y + data.default_height
  }
}

_define_
SimpleMenu (double _x, double _y)
{
  Translation pos (_x, _y)

  /*** interface ***/
  x aka pos.tx
  y aka pos.ty
  OutlineColor border_color (#000000)
  FillColor bg_color (#323232)
  FillColor text_color (#FFFFFF)
  FillColor text_selection_color (#FA9128)
  Int default_height (18)
  int default_space = 4
  String selected ("")
  List choices
  Spike fold
  Spike unfold
  /******************/

  Int width (100)
   
  FSM fsm {
    State folded
    State unfolded {
      FillColor fc (Red)
      bg_color.value =:> fc.value
      OutlineColor oc (Red)
      border_color.value =:> oc.value
      Rectangle rec_list (0, 0, 100, 0, 0, 0)
      choices.size * default_height  + default_space =:> rec_list.height
      width =:> rec_list.width
      List items
      GenericMouse.left.release -> fold
    }
    folded -> unfolded (unfold)
    unfolded -> folded (fold)
  }
   
  MaxList sum (fsm.unfolded.items, "width")
  NativeAction init (fn_init, this, 0)
  sum.output + 10 =:> width // note: +10 because in items text a draw at x=5, and we want to add 5 at the end also
}