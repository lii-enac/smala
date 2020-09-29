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
*    
*     Stephane Conversy <stephane.conversy@enac.fr>
*/

use core
use base
use display
use gui

_native_code_ 
%{
  #include <stdlib.h>

    double myrandom()
    {
        //return random()/(double)RAND_MAX; // does not work on win32
        return rand()/(double)RAND_MAX;
    }

%}

_main_
Component root {
    Frame f ("blinking", 0, 0, 600, 600)
    Exit ex (0, 1)
    f.close -> ex

    //Int delay(250)

    Translation _(100, 100)

    FillColor current_color (255, 0, 0)
    Circle light_bulb (50, 10, 20)
    FontFamily ff("B612")
    FontWeight fw(50)
    FontSize _(5,12)
    Text low_text (7, 8, "low")
    Text rpm_text (4, 18, "rpm")
            
    Switch onoff (true) {
        Component true {
            //Clock clock (250)
            FSM fsm {
                State blinkup {
                    Clock clock (250)
                    255 =: current_color.r
                }
                State blinkdown {
                    Clock clock (500)
                    100 =: current_color.r
                }
                //blinkup->blinkdown (clock.tick)
                //blinkdown->blinkup (clock.tick)
                blinkup->blinkdown (fsm.blinkup.clock.tick)
                blinkdown->blinkup (fsm.blinkdown.clock.tick)
            }
            fsm.blinkup.clock.tick -> {
                ( (myrandom() < (1/3.)) ? 50 : 0) + 500 =: fsm.blinkdown.clock.period
            }
        }

        Component false {
            100 =: current_color.r
        }
    }

    //delay =:> onoff.true.clock.period
    TextPrinter tp
    //onoff.true.clock.period =:> tp.input
    onoff.true.fsm.blinkdown.clock.period =:> tp.input

}