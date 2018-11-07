/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Stéphane Conversy <stephane.conversy@enac.fr>
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

 use core
 use base
 use gui

 import ReactiveAndCappedRefresh

 _main_
 Component root
 {
  Frame f ("redisplay", 0, 0, 1000, 1000)
  //mouseTracking = 1

  OutlineWidth ow(10)
  FillColor fc(255,0,0)
  OutlineColor _(0,0,255)
  
  Circle mobile(100, 100, 40)

  f.move.x => mobile.cx
  f.move.y => mobile.cy

  Int refresh_rate(60) // in Hz
  Int period(-1)
  1000/refresh_rate => period


  Switch test (reactive_and_capped_refresh) {
    Component default {
      1 =: DrawingRefreshManager.auto_refresh
    }
    Component default_imitation {
      0 =: DrawingRefreshManager.auto_refresh
      DrawingRefreshManager.damaged->DrawingRefreshManager.draw_sync
    }
    Component on_move {
      0 =: DrawingRefreshManager.auto_refresh
      f.move->DrawingRefreshManager.draw_sync
    }
    Component periodic {
      0 =: DrawingRefreshManager.auto_refresh
      Clock cl (-1)
      period => cl.period
      cl.tick->DrawingRefreshManager.draw_sync
    }
    Component reactive_and_capped_refresh {
      0 =: DrawingRefreshManager.auto_refresh
      ReactiveAndCappedRefresh d(f)
      refresh_rate => d.refresh_rate
    }
  }
}

run root
run syshook


