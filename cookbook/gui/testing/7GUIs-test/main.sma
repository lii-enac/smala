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

_native_code_
%{
#include <assert.h>

static
inline
double
get_double_value (const CoreProcess* p)
{
    auto * t = dynamic_cast<const AbstractProperty*>(p);
    assert(t);
    //std::cerr << floor(t->get_double_value()) << std::endl;
    return t->get_double_value();
}

%}

_main_
Component root {
    Frame f("7GUIs Test", 0, 0, 100, 100)                    // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

    Counter_7GUIs cnt(f)
    TempConverter_7GUIs tmpc(f)
    FlightBooker_7GUIs fb(f)

    mainloop -> (root) {
        // Counter
        B = find(root.cnt, "//B")
        T = find(root.cnt, "//T")

        assert (getString(T.text) == "0")

        activate (B.click)
        graph_exec()
        assert (getString(T.text) == "1")

        activate (B.click)
        graph_exec()
        assert (getString(T.text) == "2")

        // TempConverter
        TC = find(root.tmpc, "//TC")
        TF = find(root.tmpc, "//TF")

        assert (ceil(get_double_value(TC.field.content.text)) == -17)

        TC.text = "0"
        graph_exec()
        assert (floor(get_double_value(TF.field.content.text)) == 32)

        TC.text = "10"
        graph_exec()
        assert (floor(get_double_value(TF.field.content.text)) == 50)

        // FlightBooker
        C = find(root.fb, "//C")
        T1 = find(root.fb, "//T1")
        T2 = find(root.fb, "//T2")
        Book = find(root.fb, "//B")

        assert (C.value == "one-way flight")
        assert (T1.text == "01.01.26")
        assert (T2.text == "01.01.26")
        assert (Book.fsm.state == "idle")

        _DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1
        //_DEBUG_SEE_PROP_SET_VALUE = 1
        T1.init_text = "10"
        graph_exec()
        print (Book.fsm.state)
        //assert (fsm.state == "disabled")

        // end
        //deactivate (mainloop)
    }

    // _DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1
    // _DEBUG_SEE_PROP_SET_VALUE = 1


    TextPrinter tp
    B = find(root.cnt, "//B")
    B.fsm.state =:> tp.input
    fb.zog.v.B.fsm.state =:> tp.input
    // TC.text =:> tp.input


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
