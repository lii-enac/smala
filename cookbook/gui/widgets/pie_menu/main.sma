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
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

import gui.widgets.PieMenu

_main_
Component root {
  Frame frame ("cookbook piemenu", 0, 0, 1200, 400)
  Exit ex (0, 1)
  frame.close -> ex

  Component c1 {
    List items_list {
      String _  ("first")
    }

    FillColor _ (White)
    Text result (10, 50, "")

    Translation t (200, 200)
    FillColor fc (White)
    Circle icon (0, 0, 30)
    PieMenu piemenu (0, 0, 50, 150, 1)
    items_list =: piemenu.model

    "Value : " + piemenu.value =:> result.text

    FSM fsm_selection {
      State idle
      State selected {
        Component c {
          FillColor _ (Green)
          FillOpacity _acc (0.5)
          Circle mask (0, 0, 30)
        }
        "true" =: piemenu.is_open
      }
      idle -> selected (icon.release)
      selected -> idle (fsm_selection.selected.c.mask.release)
      selected -> idle (piemenu.is_open.false)
    }
  }

  Translation t2 (400, 0)

  Component c2 {
    List items_list {
      String _  ("first")
      String _ ("second")
    }

    FillColor _ (White)
    Text result (10, 50, "")

    Translation t (200, 200)
    FillColor fc (White)
    Circle icon (0, 0, 30)
    PieMenu piemenu (0, 0, 50, 150, 0)
    piemenu.text_size_px = 20
    items_list =: piemenu.model
  #AA30BB =: piemenu.background_color

    "Value : " + piemenu.value =:> result.text

    FSM fsm_selection {
      State idle
      State selected {
        Component c {
          FillColor _ (Green)
          FillOpacity _acc (0.5)
          Circle mask (0, 0, 30)
        }
        "true" =: piemenu.is_open
      }
      idle -> selected (icon.release)
      selected -> idle (fsm_selection.selected.c.mask.release)
      selected -> idle (piemenu.is_open.false)
    }
  }

  Translation t3 (400, 0)

  Component c3 {
    List items_list {
      String _  ("first")
      String _ ("second")
      String _ ("third")
      String _ ("fourth")
      String _ ("fifth")
    }

    FillColor _ (White)
    Text result (10, 50, "")

    Translation t (200, 200)
    FillColor fc (White)
    Circle icon (0, 0, 30)
    PieMenu piemenu (0, 0, 35, 100, 1)
    items_list =: piemenu.model
  #FFFFFF =: piemenu.background_color
  #AA30BB =: piemenu.foreground_color
    10 =: piemenu.text_size_px
  #000000 =: piemenu.mask_color

    "Value : " + piemenu.value =:> result.text

    FSM fsm_selection {
      State idle
      State selected {
        Component c {
          FillColor _ (Green)
          FillOpacity _acc (0.5)
          Circle mask (0, 0, 30)
        }
        "true" =: piemenu.is_open
      }
      idle -> selected (icon.release)
      selected -> idle (fsm_selection.selected.c.mask.release)
      selected -> idle (piemenu.is_open.false)
    }
  }
}

