use core
use base
use gui
use animation

import gui.animation.Animator
import gui.widgets.Button
_main_
Component root
{
  Frame f ("Ease", 0, 0, 400, 450)
  Exit ex (0, 1)
  f.close->ex

  FillColor _ (10, 10, 10)
  NoOutline _
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.{width, height} =:> bkg.{width, height}


  Button restart (f, "(re)start", 10, 10)
  Button reset (f, "reset", 10, 10)
  Button abort (f, "stop", 10, 10)
  restart.x + restart.width + 10 =:> reset.x
  reset.x + reset.width + 10 =:> abort.x
  FillColor _ (#FF1010)
  Circle c (200, 350, 10)
  
  
  // Use the same animator to avoid glitch between co-occurent animations
  Animator an (2000, 0, 1, DJN_IN_OUT_BOUNCE, 1)
  an.output * (-200) + 350 =:> c.cy
  an.output * 10 + 10 =:> c.r

  restart.click->an.start
  reset.click->an.reset
  abort.click->an.abort

  20 =: an.fps
}