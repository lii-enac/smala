use core
use base
use gui
use animation

import gui.animation.Animator
import gui.widgets.Button
_main_
Component root
{
  Frame f ("Ease", 0, 0, 400, 400)
  Exit ex (0, 1)
  f.close->ex

  FillColor _ (10, 10, 10)
  NoOutline _
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.{width, height} =:> bkg.{width, height}


  Button restart (f, "(re)start", 10, 10)
  Button reset (f, "reset", 10, 10)
  restart.x + restart.width + 10 =:> reset.x
  FillColor _ (#FF1010)
  Circle c (200, 350, 10)
  FillColor col (#1010FF)
  Circle c2 (250, 350, 10)

  Animator an2 (3000, 350, 150, DJN_OUT_ELASTIC)
  Animator an (2000, 0, 1, DJN_IN_OUT_BACK)
  an2.output =:> c.cy

  // Use the same animator to avoid glitch between co-occurent animations
  an.output * (-200) + 350 =:> c2.cy
  an.output * 10 + 10 =:> c2.r

  restart.click->an2.start
  reset.click->an.reset, an2.reset

  // Chain the animations
  an2.end->an.start

  20 =: an.fps, an2.fps
}