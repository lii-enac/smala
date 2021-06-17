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
  
  FillColor _ (#1010FF)
  Circle c2 (200, 350, 10)
  FillColor _ (#10FF10)
  Circle c3 (250, 350, 10)

  Animator an (3000, 350, 150, DJN_OUT_ELASTIC, 0, 0)
  an.output =:> c2.cy
  Animator an2 (3000, 150, 350, DJN_OUT_ELASTIC, 0, 0)
  an2.output =:> c3.cy
  
  restart.click->an.start
  reset.click->an.reset, an2.reset
  abort.click->an.abort, an2.abort

  // Chain the animations
  an.end->an2.start
  20 =: an.fps, an2.fps
}