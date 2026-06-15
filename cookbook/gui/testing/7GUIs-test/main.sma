use core
use base
use display
use gui

//import gui.DebugD
import core.property.text_property // getString
import exec_env.main_loop

import Counter_7GUIs
import TempConverter_7GUIs
import FlightBooker_7GUIs
import Timer_7GUIs
import CRUD_7GUIs
import CircleDrawer_7GUIs
import Cells_7GUIs

_native_code_
%{
#include <assert.h>
#include <math.h> // floor, ceil
using djnnstl::cerr;
using djnnstl::cout;
using djnnstl::endl;

#include "gui/picking/picking.h"

static
inline
double
get_double_value (const CoreProcess* property)
{
    auto * p = dynamic_cast<const AbstractProperty*>(property);
    assert(p);
    //std::cerr << floor(t->get_double_value()) << std::endl;
    return p->get_double_value();
}

static
inline
uint32_t
get_pixel_color (CoreProcess* frame, int x, int y)
{
    auto * f = dynamic_cast<Window*>(frame);
    assert(f);
    auto res = f->get_pixel_color(x,y);
    //puts(to_string(res).c_str());
    return res & 0xffffff;
}

static
inline
AbstractGObj*
pick_graphical_object(CoreProcess* frame, int x, int y)
{
    auto * f = dynamic_cast<Window*>(frame);
    assert(f);
    Picking * picking = f->picking_view();
    assert(picking);
    auto * picked = picking->pick(x, y);
    puts(to_string(picked).c_str());
    return dynamic_cast<AbstractGObj*>(picked);
}

%}

_main_
Component root {
    Frame f("7GUIs Tests", 0, 0, 100, 100)                    // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

    TextPrinter tp

    Component bg {
        FillColor _ (White)
        FontWeight _ (DJN_BOLD)
        Text _ (10, 50, "7GUIs Tests")
    }

    Counter_7GUIs cnt ()
    TempConverter_7GUIs tmpc ()
    FlightBooker_7GUIs fb ()
    Timer_7GUIs tmr ()
    CRUD_7GUIs crud ()
    CircleDrawer_7GUIs cd ()
    Cells_7GUIs cll ()

    mainloop -> (root) {

        // -------------------------------------------------------
        // 1- Counter
        B = find(root.cnt, "//B")
        T = find(root.cnt, "//T")

        // check initial values
        assert (getString(T.text) == "0") // the text must read "0"

        activate (B.click) // execute B action (increment)
        graph_exec()
        assert (getString(T.text) == "1") // the text should read "1"

        activate (B.click) // execute B action (increment)
        graph_exec()
        assert (getString(T.text) == "2") // the text should read "2"


        // -------------------------------------------------------
        // 2- Temp Converter
        TC = find(root.tmpc, "//TC")
        TF = find(root.tmpc, "//TF")

        assert (ceil(get_double_value(TC.field.content.text)) == -17) // the Celsius text must read "-17"
        assert (floor(get_double_value(TF.field.content.text)) == 0)  // the Fahrenheit text must read "0"

        TC.text = "0" // set Celsius Text Box to "0"
        graph_exec()
        assert (floor(get_double_value(TF.field.content.text)) == 32) // the Farenheiht text should read "32"

        TC.text = "10" // set Celsius Text Box to "10"
        graph_exec()
        assert (floor(get_double_value(TF.field.content.text)) == 50) // the Farenheiht text should read "50"

        // TODO add the other way around


        // -------------------------------------------------------
        // 3- Flight Booker
        C = find(root.fb, "//C")
        T1 = find(root.fb, "//T1")
        T2 = find(root.fb, "//T2")
        Book = find(root.fb, "//B")

        // check initial values
        assert (C.value == "one-way flight")
        assert (T1.text == "01.01.26")
        assert (T2.text == "01.01.26")
        assert (Book.fsm.state == "idle")
        //assert (pick_graphical_object(root.fb.f, 19, 47) != null) // There must be a Text here
        assert (get_pixel_color(root.fb.f, 19, 47)==0x000000) // T1 text must be black

        T1.init_text = "01.01.2" // set one-way date to an invalid date
        graph_exec()
        assert (get_pixel_color(root.fb.f, 19, 47)==0xff0000) // T1 text should be red
        print (Book.fsm.state)
        // assert (Book.fsm.state == "disabled") // button should be disabled // FIXME doesn't work
        

        // -------------------------------------------------------
        // 4- Timer
        G = find(root.tmr, "//G")   // gauge (ProgressBar) G
        L = find(root.tmr, "//L")   // label L
        S = find(root.tmr, "//S")   // slider S
        R = find(root.tmr, "//R")   // reset button R
        d = find(root.tmr, "//d")   // duration
        e = find(root.tmr, "//e")   // elapsed time
        // stop_cond = find(root.tmr, "//stop_cond")   // Stop condition

        print ("Init: Gauge = " + G.value + " -- Slider = " + S.value + " -- duration = " + d + " -- elapsed = " + e)

        // check initial values
        assert (get_double_value (G.value) == 0.0)
        assert (get_double_value (d) == get_double_value (S.value) / 10)
        assert (get_double_value (e) == 0.0)
        assert (L.text == "0.00s")

        // FXME: need something to wait ?

        activate (R.click)  // execute R action (reset)
        graph_exec()
        assert (get_double_value (G.value) == 0.0)  // The gauge value must be 0
        assert (get_double_value (e) == 0.0)        // The elapsed time must be 0
        assert (L.text == "0.00s")                  // Label should display "0.00s"
        

        // -------------------------------------------------------
        // 5- CRUD
        T_prefix = find(root.crud, "//T_prefix")
        LB = find(root.crud, "//LB")
        T_name = find(root.crud, "//T_name")
        T_surname = find(root.crud, "//T_surname")
        BC = find(root.crud, "//BC")
        BU = find(root.crud, "//BU")
        BD = find(root.crud, "//BD")

        assert (LB.items.size == 0)
        
        // Add a 1st person: "Doe, John"
        T_name.init_text = "John"
        T_surname.init_text = "Doe"
        graph_exec()
        activate (BC.click)  // execute BC action (Create)
        graph_exec()
        assert (LB.items.size == 1)

        // Add a 2nd person: "Doe, Jane"
        T_name.init_text = "Jane"
        T_surname.init_text = "Doe"
        graph_exec()
        activate (BC.click)  // execute BC action (Create)
        graph_exec()
        assert (LB.items.size == 2)

        // Add 5 more persons: "name [n], forename [n]"
        for (int n = 1; n < 6; n++) {
            T_name.init_text = "forename " + to_string (n)
            T_surname.init_text = "name " + to_string (n)
            graph_exec()
            activate (BC.click)  // execute BC action (Create)
            graph_exec()
        }
        assert (LB.items.size == 7)

        // Filter with prefix "name"
        T_prefix.init_text = "name"
        graph_exec()
        assert (LB.items.size == 5)     // 5 persons have their surname that starts with "name"

        // Reset filter
        T_prefix.init_text = ""
        graph_exec()
        assert (LB.items.size == 7)

        // Select the 6th item
        activate (LB.items.[6].bg.r.press)
        graph_exec()
        print ("T_name = " + T_name.text + " -- T_surname = " + T_surname.text)
        assert (T_name.text == "forename 4")
        // assert (T_surname.text == "name 4")  // FIXME: does NOT work

        // Rename it "name 4" --> "name 24"
        T_surname.init_text = "name 24"
        graph_exec()
        activate (BU.click)  // execute BU action (Update)
        graph_exec()

        // Update filter with prefix "name 2"
        T_prefix.init_text = "name 2"
        graph_exec()
        assert (LB.items.size == 2)     // 2 persons have their surname that starts with "name 2"

        // Reset filter
        T_prefix.init_text = ""
        graph_exec()
        
        // Select the 3st item and delete it
        activate (LB.items.[3].bg.r.press)
        graph_exec()
        activate (BD.click)  // execute BD action (Delete)
        graph_exec()
        assert (LB.items.size == 6)     // Only 6 remaining persons

        // end
        // deactivate (mainloop) // exit when done
        // print ("\033[32mall tests passed successfully\033[39;49m")
    }

    Timer wait (1000)
    // Timer wait (1100)
    wait.end -> na_wait:(root) {
        G = find(root.tmr, "//G")
        L = find(root.tmr, "//L")
        e = find(root.tmr, "//e")
        print ("After 1 s: Gauge = " + G.value + " -- elapsed = " + e + " -- label = " + L.text)
        // assert (get_double_value (G.value) == 10.0)
        // assert (get_double_value (e) == 1.0)
        // assert (L.text == "1.00s")
    }

    //_DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1
    //_DEBUG_SEE_ACTIVATION_SEQUENCE_2_MOVE = 1
    //_DEBUG_SEE_PROP_SET_VALUE = 1


    // 
    // DebugD _(f)

    // Spike good
    // Spike bad
    // Spike end

    // TextPrinter tp
    // bad -> { "bad" =: tp.input }
    // end -> { "end" =: tp.input }
    // end ->! mainloop

    

    // SwitchList sl(0) { // noloop
    //     ///t.text != 0 -> bad
    //     //t.text != 100 -> bad
    //     //getString(t.text) =:> tp.input // FIXME EEEEEEE !!!! // WARNING : smalac may generates more than 1 process per "statement":
    //     //Component _
    //     Component _ {
    //         getString(t.text) != "0.0" -> bad
    //     }
    //     Component _ {
    //         getString(t.text) != "1" -> bad
    //     }
    //     Component _ {
    //         getString(t.text) != "2" -> bad
    //     }
    //     Component _ {
    //         |-> end
    //     }
    // }
    // sl.index =:> tp.input

     

    //Clock c(10)
    
    //c.tick -> b.click
    //c.tick -> sl.next
    //f.press
    // mainloop -> (root) {
    //     while (1) {
    //     activate (root.b.click)
    //     activate (root.sl.next)
    //     graph_exec()
    //     }
    // }
    

    //f.press -> sl.next
    //f.release -> b.click
    //b.click -> sl.next
    //f.press -> b.press
    //f.release -> b.release
    //f.release -> b.click
}




// [7GUIs] The task is to build a frame containing a label or read-only textfield T and a button B.
// [7GUIs] Initially, the value in T is “0” and each click of B increases the value in T by one.

// [7GUIs] Counter serves as a gentle introduction to the basics of the language, paradigm and toolkit
// [7GUIs] for one of the simplest GUI applications imaginable.
// [7GUIs] Thus, Counter reveals the required scaffolding and how the very basic features work together to build a GUI application.

// [7GUIs] A good solution will have almost no scaffolding.
