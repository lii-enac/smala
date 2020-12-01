/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Stephane Conversy <stephane.conversy@enac.fr>
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

_main_
Component root {
    Frame f ("f", 0, 0, 500, 600)
    Exit ex (0, 1)
    f.close -> ex
    mouseTracking = 1

    IntProperty i(0)
    TextProperty t("t")
    TextPrinter tp
    i =:> tp.input
    t =:> tp.input

    // WARNING : smalac may generates more than 1 process per "statement":
    // two or more activations of SwitchList.next might be required in the following to 'switch' to an instruction 
    // let the authors/maintainers know if you find such a case 
    SwitchList imperative_statements {
        1 =: i
        f.move.x =:> i
        3 =: i
        2+4 =: i
        "foo" =: t
        Circle _(20,20,10)
        Circle _(80,20,10)
        5 =: i
        Component _ {
            8 =: i
            "bar" =: t
        }
    }

    // PC = Program Counter
    PCincr aka imperative_statements.next
    //PCincr aka imperative_statements.previous // upside PC, even more cryptic than befunge ;-)
    // seriously the above ^^ demonstrates how order is explicit (Y visual variable), but direction is not => cf LangViz paper
    f.press -> PCincr

    // simulating a 1hz computer
    //Clock cl(1000)
    //cl.tick -> PCincr

    // other conditions to incr the PC
    //i>200 -> PCincr // try it when f.move.x =:> i is active
}
