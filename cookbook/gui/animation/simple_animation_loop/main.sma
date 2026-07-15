/*
*  Smala cookbook simple_animation_loop
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*
*/
use core
use base
use gui
use animation

import gui.animation.Animator

import gui.widgets.StandAlonePushButton
import gui.widgets.StandAloneToggleButton
import gui.widgets.StandAloneComboBox





_main_
Component root
{
  Frame f ("Ease", 0, 0, 600, 400)
  f.background_color.r = 10
	f.background_color.g = 10
	f.background_color.b = 10

  // Allows to hover the arrow of the combobox
  mouseTracking = 1

  Exit ex (0, 1)
  f.close->ex

  TextPrinter tp

  Component upper_layer

  ProcessCollector collection

  List lst {
    StandAlonePushButton btn_start ("(re)start", 1, 1)
    StandAlonePushButton btn_abort ("stop", 1, 1)
    StandAlonePushButton btn_pause ("pause", 1, 1)
    StandAlonePushButton btn_resume ("resume", 1, 1)
    StandAlonePushButton btn_reset ("reset", 1, 1)
    StandAlonePushButton btn_rewind ("rewind", 1, 1)
    StandAloneToggleButton btn_activate ("activate", 1, 1)
  }

  btn_start aka lst[1]
  btn_abort aka lst[2]
  btn_pause aka lst[3]
  btn_resume aka lst[4]
  btn_reset aka lst[5]
  btn_rewind aka lst[6]
  btn_activate aka lst[7]

  for (int i = 2; i <= lst.size; i++) {
    lst[i-1].x + lst[i-1].width + 15 =:> lst[i].x
  }

  // --------------------------------
  // 1- Loop (and start when user clicks on the corresponding button)
  // --------------------------------
  FillColor _ (#FF1010)
  Circle c1 (50, 350, 10)
  
  // Use the same animator to avoid glitch between co-occurent animations
  Animator anim1 (2000, 0, 1, DJN_IN_OUT_BOUNCE, 1, 0)
  anim1.output * (-200) + 350 =:> c1.cy
  anim1.output * 10 + 10 =:> c1.r


  // --------------------------------
  // 2- Loop and start on activation (at launch of our window)
  // --------------------------------
  FillColor _ (#10FF10)
  Circle c2 (200, 350, 10)

  // Use the same animator to avoid glitch between co-occurent animations
  Animator anim2 (2000, 0, 1, DJN_IN_OUT_BOUNCE, 1, 1)
  anim2.output * (-200) + 350 =:> c2.cy
  anim2.output * 10 + 10 =:> c2.r


  20 =: anim1.fps, anim2.fps

  Bool is_activated (false)
  btn_activate.toggle -> {
    !is_activated =: is_activated
  }


  // --------------------------------
  // 3- Two animations that running alternatively
  // --------------------------------

  Int BREATHE_DURATION_MS (2000)

  Switch switch1 (false) {
    Component false

    Component true {
      FillColor _ (#1010FF)
      Circle c3 (350, 350, 10)

      FSM fsm {
        State breathe_in {
          Animator anim_in ($BREATHE_DURATION_MS, 0, 1, DJN_IN_OUT_SINE, 0, 1)
          anim_in.output * (-200) + 350 =:> c3.cy
          anim_in.output * 10 + 10 =:> c3.r
        }

        State breathe_out {
          Animator anim_out ($BREATHE_DURATION_MS, 1, 0, DJN_IN_OUT_SINE, 0, 1)
          anim_out.output * (-200) + 350 =:> c3.cy
          anim_out.output * 10 + 10 =:> c3.r
        }

        breathe_in -> breathe_out (breathe_in.anim_in.end)
        breathe_out -> breathe_in (breathe_out.anim_out.end)
      }
    }
  }
  is_activated =:> switch1.state


  btn_start.click -> anim1.start, anim2.start
  btn_abort.click -> anim1.abort, anim2.abort
  btn_pause.click -> anim1.pause, anim2.pause
  btn_resume.click -> anim1.resume, anim2.resume
  btn_reset.click -> anim1.reset, anim2.reset
  btn_rewind.click -> anim1.rewind, anim2.rewind


  // --------------------------------
  // 4- Animation when mouse enters/leave arrow background
  // --------------------------------
  StandAloneComboBox combo (upper_layer, "", 450, 100)
  addChildrenTo combo.str_items {
    String _ ("item 1")
    String _ ("item 2")
    String _ ("item 3")
    String _ ("item 4")
  }

  // For combobox
  //moveChild upper_layer >>
}