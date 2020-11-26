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

    // FIXME : smalac generates 2 process per assignment: two presses are required in the following to 'scitch' to an assignment 
    // FIXME : possible solution: set null to parent of the generated DoubleProperty before the assignement in smalac
    SwitchList imperative_statements {
        1 =: i
        f.move.x =:> i
        3 =: i
        Circle _(20,20,10)
        Circle _(80,20,10)
        4 =: i
    }

    PCincr aka imperative_statements.next
    //PCincr aka imperative_statements.previous // more cryptic than befunge ;-)
    f.press -> PCincr

    TextPrinter tp
    i =:> tp.input
}
