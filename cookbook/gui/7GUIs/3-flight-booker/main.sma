// [7GUIs] Flight Booker
// [7GUIs] https://eugenkiss.github.io/7guis/tasks#flight
// [7GUIs] Challenge: Constraints.


use core
use base
use display
use gui

import gui.widgets.PushButton
import gui.widgets.UITextField
import gui.widgets.VBox
import gui.widgets.ComboBox

_native_code_
%{
    // #include <iostream>
    // #include "core/property/s_to_p.h"

    int to_time(const djnnstl::string& yys, const djnnstl::string& mms, const djnnstl::string& dds ) {
        int yy, mm, dd;
        yy = atoi(yys.c_str());
        mm = atoi(mms.c_str());
        dd = atoi(dds.c_str());
        //std::cerr << "to_time " << yy << " " << mm << " " << dd << std::endl;
        std::tm tm{};
        if (yy<30) yy += 2000; // try to be...
        else if (yy>30) yy += 1900; // ...smart (?)
        tm.tm_year = yy - 1900;
        tm.tm_mon = mm - 1;
        tm.tm_mday = dd;
        std::time_t t = std::mktime(&tm);
        return t;
    }

    // int to_time(AbstractSimpleProperty * yys_, AbstractSimpleProperty * mms_, AbstractSimpleProperty * dds_) {
    //     auto yys = dynamic_cast<TextProperty*>(yys_);
    //     auto mms = dynamic_cast<TextProperty*>(mms_);
    //     auto dds = dynamic_cast<TextProperty*>(dds_);
    //     return to_time (yys->get_value(), mms->get_value(), dds->get_value());
    // }
%}

_main_
Component root {
    Frame f ("7GUIs Flight Booker - DOES NOT WORK YET") // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

    ComboBox _C                 // [7GUIs] a combobox C
    UITextField _T1             // [7GUIs] two textfields T1
    UITextField _T2             // [7GUIs] and T2 representing the start and return date, respectively,
    PushButton _B("Book")       // [7GUIs] and a button B for submitting the selected flight.

    VBox h(f) {}
    addChildrenTo h.items {
        _C,
        _T1,
        _T2,
        _B
    }
    // FIXME now that all widgets have been reparented by addChildrenTo, they are inaccessible :-/
    C  aka h.items.[1]
    T1 aka h.items.[2]
    T2 aka h.items.[3]
    B  aka h.items.[4]
    
    Component model {           // [7GUIs] with the two options “one-way flight” and “return flight”,
        List items {
          String one_way ("one-way flight")
          String return_flight ("return flight")
        }
    }
    model =: C.model
    C.preferred_width = 100
    
    // FIXME why is the combobox not inited on value 1 ?
    model.items.[1] =: C.value                  // [7GUIs] Initially, C has the value “one-way flight”
    // FIXME the text fields shrink when selecting an entry in the combo box !!!!???

    // FIXME does not work
    //"01/01/2025" =:> T1.text //= "01/01/2025"   // [7GUIs] and T1 as well as T2 have the same (arbitrary) date
    //T2.text = "01/01/2025"
                                                // [7GUIs] (it is implied that T2 is disabled). (see below)


    |-> B.disable // FIXME? -:>                 // [7GUIs] and B is disabled.
    |-> T2.disable // FIXME T2 non-editing mode resembles disabled state
    
    Bool one_way(0)
    C.value == "one-way flight" =:> one_way
    // FIXME could be model.items[1] or .one_way -> T2 .return_flight ->! T2
    one_way.true  -> T2.disable                 // [7GUIs] T2 is enabled iff C’s value is “return flight”.
    one_way.false -> T2.enable                  // [7GUIs] T2 is enabled iff C’s value is “return flight”.

    
    // [7GUIs] (e.g. “You have booked a one-way flight on 04.04.2014.”).
    
    TextPrinter tp
    B.click -> {                                                             // [7GUIs] When clicking B 
        "You have booked a " + C.value + " flight on " + T1.text =: tp.input // [7GUIs] a message is displayed informing the user of his selection
        // FIXME? popup?
    }

    
    // [7GUIs] FIXME the following validates the date even if the field is disabled

    Bool T1_valid(0)
    Bool T2_valid(0)
    Bool B_valid(0)

    Int time1(-1)

    Component _ {

        Regex regex_date ("\\s*(\\d\\d?)\\s(\\d\\d?)\\s(\\d\\d)\\s*")

        T1.text => regex_date.input

        String yy("0")
        String mm("0")
        String dd("0")
        regex_date.[1] =:> dd // FIXME? regex_date[1] (without the .) is accepted by the compiler 
        regex_date.[2] =:> mm
        regex_date.[3] =:> yy

        //dd+" " + mm + " " + yy =:> tp.input

        Bool res(0)
        regex_date.matched =:> res // FIXME bool property find_child_impl
        res.true -> {
            //to_time($toString(regex_date.[1]), $toString(regex_date.[2]), $toString(regex_date.[3])) =: time1 // FIXME Native expression action
            to_time($getString(yy), $getString(mm), $getString(dd)) =: time1
        }
        res.false -> {
            -1 =: time1
        }
        //time1 => tp.input
        time1 != -1 =:> T1_valid

        T1_valid.false -> {
            #FF0000 =: T1.text_color                // [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red
        }
        T1_valid.true -> {
            #000000 =: T1.text_color
        }
    }

    Switch T2_sw(disabled) {
        Component disabled {
            T1_valid =:> B_valid
        }
        Component enabled {
            //regex_date_2 = clone(regex_date) // FIXME clone does not work
            Regex regex_date_2 ("\\s*(\\d\\d?)\\s(\\d\\d?)\\s(\\d\\d)\\s*")
            T2.text =:> regex_date_2.input
            Int time2(-1)
            Bool res(0)
            regex_date_2.matched =:> res
            //regex_date_2.matched == 1 -> { // FIXME?
            //regex_date_2.matched.true -> { // FIXME find_child_impl in BoolProperty
            String yy("")
            String mm("")
            String dd("")
            regex_date_2.[1] =:> dd
            regex_date_2.[2] =:> mm
            regex_date_2.[3] =:> yy
            res.true -> {
                //to_time($getString(regex_date_2.[1]), $getString(regex_date_2.[2]), $getString(regex_date_2.[3])) =: time2
                to_time($getString(yy), $getString(mm), $getString(dd)) =: time2
                //to_time(regex_date_2.[1], regex_date_2.[2], regex_date_2.[3]) =: time2
                //T2.text + " /" + regex_date_2.[1] + " " + regex_date_2.[2] + " " + regex_date_2.[3] + "/ " + time2 =: tp.input
                //regex_date_2.[2] =: tp.input
            }
            //regex_date_2.matched.false -> {
            res.false -> {
                -1 =: time2
            }
            //T2.text + "time2 " + time2 =:> tp.input
            time2 != -1 =:> T2_valid

            T2_valid.false -> {
                #FF0000 =: T2.text_color                // [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red
            }
            T2_valid.true -> {
                #000000 =: T2.text_color
            }

            T1_valid && T2_valid && (time1 <= time2) =:> B_valid
        }
    }
                 
    one_way.true -> {
        "disabled" =: T2_sw.state 
    }
    one_way.false -> {
        "enabled" =: T2_sw.state 
    }
    //T2_sw.state =:> tp.input


    // [7GUIs] When C has the value “return flight” and T2’s date is strictly before T1’s then B is disabled.
    // FIXME the following does not work if C is "one-way":
    B_valid.true -> B.enable // FIXME should be -:>
    B_valid.false -> B.disable
    //"--\nT1_valid " + T1_valid + "\nT2_valid " + T2_valid + "\nB_valid " + B_valid =:> tp.input
}



// [7GUIs] The task is to build a frame containing a combobox C with the two options “one-way flight” and “return flight”,
// [7GUIs] two textfields T1 and T2 representing the start and return date, respectively,
// [7GUIs] and a button B for submitting the selected flight.
// [7GUIs] T2 is enabled iff C’s value is “return flight”.
// [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red and B is disabled.
// [7GUIs] When C has the value “return flight” and T2’s date is strictly before T1’s then B is disabled.

// [7GUIs] The focus of Flight Booker lies on modelling constraints between widgets on the one hand
// [7GUIs] and modelling constraints within a widget on the other hand.
// [7GUIs] Such constraints are very common in everyday interactions with GUI applications.

// [7GUIs] A good solution for Flight Booker will make the constraints clear, succinct and explicit in the source code and not hidden behind a lot of scaffolding.

// [7GUIs] Flight Booker is directly inspired by the Flight Booking Java example in Sodium with the simplification of using textfields for date input instead of specialized date picking widgets as the focus of Flight Booker is not on specialized/custom widgets.

