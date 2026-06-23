/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*/

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

/* ---------------------------------------------------------------------------------------------
 *  In order to test all 7GUIs app, this app must be able to control your mouse.
 *  We use the lib "cpp auto gui" to move the cursor & to press/release the mouse buttons.
 *  To allow the lib, you have to go into "System Settings" (or System "Preferences").
 *  Then "Privacy & Security", "Accessibility" and enable "Terminal" in the list of applications
 * --------------------------------------------------------------------------------------------- */

_native_code_
%{
    #include <assert.h>
    #include <math.h> // floor, ceil
    using djnnstl::cerr;
    using djnnstl::cout;
    using djnnstl::endl;

    #include "gui/picking/picking.h"

    #include "../../cppautogui/include/cppautogui.hpp"
    using cppautogui::Point;
    using cppautogui::MouseButton;

    void move_mouse_to (double x, double y) {
        move_to (Point {x, y}, 0.0);
    }

    void press_at (double x, double y, int button) {
        switch (button) {
            case 0:
                mouse_down (Point {x, y}, MouseButton::Left);
                break;
            case 1:
                mouse_down (Point {x, y}, MouseButton::Right);
                break;
            case 2:
                mouse_down (Point {x, y}, MouseButton::Middle);
                break;
            default:
                break;
        }
    }

    void release_at (double x, double y, int button) {
        switch (button) {
            case 0:
                mouse_up (Point {x, y}, MouseButton::Left);
                break;
            case 1:
                mouse_up (Point {x, y}, MouseButton::Right);
                break;
            case 2:
                mouse_up (Point {x, y}, MouseButton::Middle);
                break;
            default:
                break;
        }
    }

    // void drag_mouse_to (double x, double y) {
    //     drag_to (Point {x, y}, MouseButton::Left, 1.0);
    // }
    

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
        // FIXME: assert get_pixel_color
        //assert (pick_graphical_object(root.fb.f, 19, 47) != null) // There must be a Text here
        // assert (get_pixel_color(root.fb.f, 19, 47)==0x000000) // T1 text must be black

        T1.init_text = "01.01.2" // set one-way date to an invalid date
        graph_exec()
        // FIXME: assert get_pixel_color
        // assert (get_pixel_color(root.fb.f, 19, 47)==0xff0000) // T1 text should be red
        print (Book.fsm.state)
        // assert (Book.fsm.state == "disabled") // button should be disabled // FIXME doesn't work
        

        // -------------------------------------------------------
        // 4- Timer
        G = find(root.tmr, "//G")   // gauge (ProgressBar) G
        Lbl = find(root.tmr, "//Lbl")   // label L
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
        assert (Lbl.text == "0.00s")

        // FXME: need something to wait ?
        

        // -------------------------------------------------------
        // 5- CRUD
        T_prefix = find(root.crud, "//T_prefix")
        L = find(root.crud, "//L")
        T_name = find(root.crud, "//T_name")
        T_surname = find(root.crud, "//T_surname")
        BC = find(root.crud, "//BC")
        BU = find(root.crud, "//BU")
        BD = find(root.crud, "//BD")

        assert (L.items.size == 0)
        
        // Add a 1st person: "Doe, John"
        T_name.init_text = "John"
        T_surname.init_text = "Doe"
        graph_exec()
        activate (BC.click)  // execute BC action (Create)
        graph_exec()
        assert (L.items.size == 1)

        // Add a 2nd person: "Doe, Jane"
        T_name.init_text = "Jane"
        T_surname.init_text = "Doe"
        graph_exec()
        activate (BC.click)  // execute BC action (Create)
        graph_exec()
        assert (L.items.size == 2)

        // Add 5 more persons: "name [n], forename [n]"
        for (int n = 1; n < 6; n++) {
            T_name.init_text = "forename " + to_string (n)
            T_surname.init_text = "name " + to_string (n)
            graph_exec()
            activate (BC.click)  // execute BC action (Create)
            graph_exec()
        }
        assert (L.items.size == 7)

        // Filter with prefix "name"
        T_prefix.init_text = "name"
        graph_exec()
        assert (L.items.size == 5)     // 5 persons have their surname that starts with "name"

        // Reset filter
        T_prefix.init_text = ""
        graph_exec()
        assert (L.items.size == 7)

        // Select the 6th item
        activate (L.items.[6].bg.r.press)
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
        assert (L.items.size == 2)     // 2 persons have their surname that starts with "name 2"

        // Reset filter
        T_prefix.init_text = ""
        graph_exec()
        
        // Select the 3st item and delete it
        activate (L.items.[3].bg.r.press)
        graph_exec()
        activate (BD.click)  // execute BD action (Delete)
        graph_exec()
        assert (L.items.size == 6)     // Only 6 remaining persons


        // -------------------------------------------------------
        // 6- Circle Drawer
        canvas = find(root.cd, "//canvas")
        undo = find(root.cd, "//undo")
        redo = find(root.cd, "//redo")

        move_mouse_to (0, 0)

        // Too soon --> we have to wait
        // activate (canvas.mask.left.press)
        // graph_exec()
        // move_mouse_to (110, 520)
        // press_at (110, 520, 0)


        // -------------------------------------------------------
        // 7- Cells
        cells = find(root.cll, "//cells")
        edit = find(root.cll, "//box_edit")

        B2 = find (cells, "22")
        B2.value = "100"

        C2 = find (cells, "23")
        C2.value = "100"

        B3 = find (cells, "32")
        B3.value = "50"

        C3 = find (cells, "33")
        C3.value = "0"

        B4 = find (cells, "42")
        B4.value = "100"

        C4 = find (cells, "43")
        C4.value = "100"

        print ("B2=" + B2.value + " -- C2=" + C2.value + " -- B3=" + B3.value + " -- C3=" + C3.value + " -- B4=" + B4.value + " -- C4=" + C4.value)

        B6 = find (cells, "62")
        activate (B6.bg.press)  // Set as current cell
        graph_exec()
        B6.formula = "=sum(B2:B4)"
        graph_exec()
        // print ("B6=" + B6.value)
        assert (B6.value == "250")

        C6 = find (cells, "63")
        activate (C6.bg.press)  // Set as current cell
        graph_exec()
        C6.formula = "=avg(C2:C4)"
        graph_exec()
        // print ("C6=" + C6.value)
        assert (C6.value == "66")

        // end
        // deactivate (mainloop) // exit when done
        // print ("\033[32mall tests passed successfully\033[39;49m")
    }


    // -------------------------------------------------------
    // Callbacks

    Int nb_press_on_canvas (0)

    na_canvas_pressed = find(root.cd, "//na_canvas_pressed")
    na_canvas_pressed -> na_CD_canvas_pressed:(root) {
        canvas = find(root.cd, "//canvas")

        root.nb_press_on_canvas++
        print (root.nb_press_on_canvas + " press on canvas = " + canvas.items.size + " drawn circles ?")
        assert (floor (get_double_value (canvas.items.size)) == root.nb_press_on_canvas)
    }


    // -------------------------------------------------------
    // Timers to wait...

    Timer wait_500ms (500)
    wait_500ms.end -> na_wait_500ms:(root) {
        G = find(root.tmr, "//G")   // gauge (ProgressBar) G
        Lbl = find(root.tmr, "//Lbl")   // label L
        // S = find(root.tmr, "//S")   // slider S
        R = find(root.tmr, "//R")   // reset button R
        // d = find(root.tmr, "//d")   // duration
        e = find(root.tmr, "//e")   // elapsed time

        activate (R.click)  // execute R action (reset)
        graph_exec()
        print ("RESET timer: Gauge = " + G.value + " -- elapsed = " + e + " -- label = " + Lbl.text)
        assert (get_double_value (G.value) == 0.0)  // The gauge value must be 0
        // assert (get_double_value (e) == 0.0)        // The elapsed time must be 0
        // assert (Lbl.text == "0.00s")                // Label should display "0.00s"
    }

    Timer wait_1_5s (1500)
    wait_1_5s.end -> na_wait_1_5s:(root) {
        // Create 1st circle
        move_mouse_to (100, 500)
        press_at (100, 500, 0)
    }

    Timer wait_2s (2000)
    wait_2s.end -> na_wait_2s:(root) {
        // Create 2nd circle
        move_mouse_to (250, 800)
        press_at (250, 800, 0)
    }

    Timer wait_2_5s (2500)
    wait_2_5s.end -> na_wait_2_5s:(root) {
        // Create 3rd circle
        move_mouse_to (400, 650)
        press_at (400, 650, 0)
    }

    Timer wait_3s (3000)
    wait_3s.end -> na_wait_3s:(root) {
        // 2,5 sec since we reseted the timer

        G = find(root.tmr, "//G")
        Lbl = find(root.tmr, "//Lbl")
        e = find(root.tmr, "//e")
        print ("After 2.5 sec: Gauge = " + G.value + " -- elapsed = " + e + " -- label = " + Lbl.text)
        // assert (get_double_value (G.value) == 25.0)
        // assert (get_double_value (e) == 2.5)
        // assert (Lbl.text == "2.50s")
    }

    na_wait_3s -> (root) {
        canvas = find(root.cd, "//canvas")
        undo = find(root.cd, "//undo")
        redo = find(root.cd, "//redo")
        radius_of_selected = find(root.cd, "//_radius_of_selected_item")

        print (canvas.items.size + " drawn circles (after 3 sec.)")
        assert (floor (get_double_value (canvas.items.size)) == 3)

        // Select 2nd circle & right click to open the menu
        move_mouse_to (250, 800)
        press_at (250, 800, 1)      // MouseButton::Right

        print ("(default) radius of selected circle = " + radius_of_selected.value)
    }

    Timer wait_3_5s (3500)
    wait_3_5s.end -> na_wait_3_5s:(root) {
        // Move hover option "Adjust diameter"...
        move_mouse_to (270, 810)
        // ... and click on this option
        press_at (270, 810, 0)
        release_at (270, 810, 0)
    }

    Timer wait_4s (4000)
    wait_4s.end -> na_wait_4s:(root) {
        radius_slider = find(root.cd, "//radius_slider")
        popup_slider = find(root.cd, "//popup_slider")
        
        // Update radius
        radius_slider.output = 45
        graph_exec ()

        // FIXME: Crash !
        // activate (popup_slider.f2.close)
        // graph_exec ()

        // Click on close button (of frame 2)
        move_mouse_to (545, 714)
        press_at (545, 714, 0)
        release_at (545, 714, 0)
    }

    Timer wait_4_5s (4500)
    wait_4_5s.end -> na_wait_4_5s:(root) {
        canvas = find(root.cd, "//canvas")
        undo = find(root.cd, "//undo")
        redo = find(root.cd, "//redo")
        radius_of_selected = find(root.cd, "//_radius_of_selected_item")

        print ("(increased) radius of selected circle = " + radius_of_selected.value)
        assert (floor (get_double_value (radius_of_selected.value)) == 45)

        activate (undo.click)  // execute undo action
        graph_exec()
        assert (floor (get_double_value (radius_of_selected.value)) == 10)
        print ("(reseted) radius of selected circle = " + radius_of_selected.value + " (after 1st undo)")

        // FIXME: NO refresh of UI

        activate (undo.click)  // execute undo action
        graph_exec()
        assert (floor (get_double_value (canvas.items.size)) == 2)

        activate (undo.click)  // execute undo action
        graph_exec()
        assert (floor (get_double_value (canvas.items.size)) == 1)

        print (canvas.items.size + " drawn circles (after 3 undo)")
    }

    Timer wait_5s (5000)
    wait_5s.end -> na_wait_5s:(root) {
        canvas = find(root.cd, "//canvas")
        undo = find(root.cd, "//undo")
        redo = find(root.cd, "//redo")
        radius_of_selected = find(root.cd, "//_radius_of_selected_item")

        activate (redo.click)  // execute redo action
        graph_exec()
        assert (floor (get_double_value (canvas.items.size)) == 2)

        activate (redo.click)  // execute redo action
        graph_exec()
        assert (floor (get_double_value (canvas.items.size)) == 3)

        print (canvas.items.size + " drawn circles (after 2 redo)")

        activate (redo.click)  // execute redo action
        graph_exec()
        print ("radius of selected circle = " + radius_of_selected.value + " (after 3 redo)")
        assert (floor (get_double_value (radius_of_selected.value)) == 45)
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
