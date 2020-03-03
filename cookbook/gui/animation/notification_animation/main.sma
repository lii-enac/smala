/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2017)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Jérémie Garcia <jeremie.garcia@enac.fr>
 *
 */
use core
use base
use display
use gui

_native_code_ 
%{
  #include <cmath>
%}

_main_
Component root {
  Frame f ("Notification animation", 0, 0, 300, 120)
  Exit ex (0, 1)
  f.close -> ex

  FillColor _ (100, 250, 100)
  Rectangle start_r (50,50, 60, 20, 5, 5)
  FillColor _ (0, 0, 0)
  Text start_txt (60, 66, "trigger")

  //animation
  Incr radius_inc (1)
  Bool res (0)

  //stop at 60
      AssignmentSequence reset (1){
        0 =: radius_inc.state
      }

      

  FSM animateFSM{
    State stopped{
    }
    State running{
      Double fps (50)
      Clock clock (50)
      1000 / fps =: clock.period
      clock.tick -> radius_inc

      
      (radius_inc.state > 70) => res
      res.true -> reset

      FillColor _ (209, 200, 27)
      Ellipse source (200, 65, 0, 0)

      NoFill _
      OutlineOpacity ellipseOp (1)
      OutlineColor _ (209, 200, 27)
      Ellipse anim1 (200, 65, 0, 0)

      //animation SwitchRange
      int step = 20
      SwitchRange animationSteps ($radius_inc){
        //circle appears
        _ [0, 20[{
            radius_inc.state => source.rx, source.ry 
          
        }
        //circle disappear and ellipses arrive
        _ [20, 30[{

              radius_inc.state => anim1.rx, anim1.ry
          (1-(radius_inc.state-20) / 10) => ellipseOp.a
        }
          //circle disappear and ellipses arrive
        _ [30, 40[{

                    radius_inc.state -10 => anim1.rx, anim1.ry
          (1-(radius_inc.state-30) / 10) => ellipseOp.a
        }

          //circle disappear and ellipses arrive
        _ [40, 50[{

              fmod((radius_inc.state - 20) , 5) => anim1.rx, anim1.ry
          (1-(radius_inc.state-40) / 30) => ellipseOp.a
        }
        //decrease source
        _ [50, 60]{

          0 =: anim1.rx, anim1.ry
          60 - radius_inc.state => source.rx, source.ry
        }
        //pause
        _ [60, 70]{
        }
      }
      radius_inc.state => animationSteps.input
    }
    stopped -> running (start_r.press)
    running -> stopped (res.true)
  }
}

run root
run syshook
