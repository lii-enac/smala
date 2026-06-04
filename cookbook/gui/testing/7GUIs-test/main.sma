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
    b_ = find(cnt, "//B")
    t_ = find(cnt, "//T")
    B aka b_
    T aka t_

    TempConverter_7GUIs tmpc(f)
    tc_ = find(tmpc, "//TC")
    tf_ = find(tmpc, "//TF")
    TC aka tc_
    TF aka tf_

    FlightBooker_7GUIs fb(f)
    c_ = find(fb, "//C")
    t1_ = find(fb, "//T1")
    t2_ = find(fb, "//T2")
    book_ = find(fb, "//B")
    C aka c_
    T1 aka t1_
    T2 aka t2_
    Book aka book_
    fsm aka Book.fsm

    mainloop -> (root) {
        // Counter
        assert (getString(root.T.text) == "0")

        activate (root.B.click)
        graph_exec()
        assert (getString(root.T.text) == "1")

        activate (root.B.click)
        graph_exec()
        assert (getString(root.T.text) == "2")

        // TempConverter
        assert (ceil(get_double_value(root.TC.field.content.text)) == -17)

        root.TC.text = "0"
        graph_exec()
        assert (floor(get_double_value(root.TF.field.content.text)) == 32)

        root.TC.text = "10"
        graph_exec()
        assert (floor(get_double_value(root.TF.field.content.text)) == 50)

        // FlightBooker
        assert (root.C.value == "one-way flight")
        assert (root.T1.text == "01.01.26")
        assert (root.T2.text == "01.01.26")
        assert (root.fsm.state == "idle")

        _DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1
        //_DEBUG_SEE_PROP_SET_VALUE = 1
        root.T1.init_text = "10"
        graph_exec()
        print (root.Book.fsm.state)
        //assert (root.fsm.state == "disabled")

        // end
        //deactivate (mainloop)
    }

    // _DEBUG_SEE_ACTIVATION_SEQUENCE_2 = 1
    // _DEBUG_SEE_PROP_SET_VALUE = 1


    TextPrinter tp
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
