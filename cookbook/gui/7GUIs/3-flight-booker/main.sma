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
    //#include <iostream>
    int 
    to_time(double yy_d, double mm_d, double dd_d) {
   
    //std::cerr << "to_time yy:" << yy_d << " mm:" << mm_d << " dd:" << dd_d << std::endl;

    // 2. Conversion des double en int
    int yy = static_cast<int>(yy_d);
    int mm = static_cast<int>(mm_d);
    int dd = static_cast<int>(dd_d);

    if (mm < 1 || mm > 12 || dd < 1 || dd > 31) return -1;

    const int max_day[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (dd > max_day[mm - 1]) {
        if (!(mm == 2 && dd == 29 && yy % 4 == 0)) {
            return -1;
        }
    }

    std::tm tm{};
    tm.tm_year = yy + 100; // because the yy start is 1900
    tm.tm_mon = mm - 1;
    tm.tm_mday = dd;
    tm.tm_isdst = -1; // Indispensable pour éviter les erreurs de changement d'heure
    std::time_t t = std::mktime(&tm);
    
    //std::cerr << "t: " << t << std::endl;
    return static_cast<int>(t);
}
%}

_main_
Component root {
    Frame f ("7GUIs Flight Booker", 800, 500, 150, 120) // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

    ZOrderedGroup zog {
        VBox v {    
            ComboBox C             // [7GUIs] ...a combobox C...
            UITextField T1         // [7GUIs] ...two textfields T1...
            UITextField T2         // [7GUIs] ...and T2 representing the start and return date, respectively,...
            PushButton B("Book")   // [7GUIs] ...and a button B for submitting the selected flight.
        }
    }
    // make widget naming independent from layout hierarchy
    box aka zog.v
    C  aka box.C
    T1 aka box.T1
    T2 aka box.T2
    B  aka box.B

    Component cb_model {           // [7GUIs] ...with the two options “one-way flight” and “return flight”...
        List items {
          String one_way ("one-way flight")
          String return_flight ("return flight")
        }
    }
    cb_model =: C.model
    // C.preferred_width = 120
    
    cb_model.items.[1] =: C.value  // [7GUIs] Initially, C has the value “one-way flight”...

    "01.01.26" =: T1.init_text  // [7GUIs] ...and T1 as well as T2 have the same (arbitrary) date...
    "01.01.26" =: T2.init_text
    
    Bool is_return_flight(0)
    is_return_flight.true  -> T2.enable     // [7GUIs] T2 is enabled iff [if...]
    is_return_flight.false -> T2.disable    // [7GUIs] [... and only if] C’s value is “return flight”.
    (C.value == "return flight") =:> is_return_flight
    
    TextPrinter tp // FIXME popup? or a text beneath the Vbox
    B.click -> (root) {   // [7GUIs] When clicking B...
        if (root.is_return_flight) {
            root.tp.input = "You have booked a " + root.C.value + " on " + root.T1.text + " return on " + root.T2.text
        }
        else {                                                      
            root.tp.input = "You have booked a " + root.C.value + " on " + root.T1.text      // [7GUIs] ...a message is displayed informing the user of his selection 
        }                                                                                    // [7GUIs] (e.g. “You have booked a one-way flight on 04.04.2014.”).
    }                                                                        


    Bool T1_valid(0)
    Bool T2_valid(0)
    Bool  B_valid(0)

    Int time1(-1)
    Int time2(-1)

    // 3 types of regex
    // string regex_date_str = "\\s*(\\d\\d?)\\s(\\d\\d?)\\s(\\d\\d)" // FIXME group name (not in c++ regex sadly)
    // Regex regex_date ("\\s*(\\d\\d?)\\s(\\d\\d?)\\s(\\d\\d)\\s*")
        
    // Matches dates in the formats: DD.MM.YY, DD.MM.YYYY, DD MM YY, or DD MM YYYY
    // - Day and month: 1 or 2 digits
    // - Year: exactly 2 or 4 digits (3-digit years are rejected)
    // - Separator: either a single dot or a single space
    // - The same separator must be used consistently between all parts
    // - No extra spaces allowed between the date components (only around the whole date)
    // Valid examples: 10.02.26, 1 9 2026
    // Invalid examples: 10. 02. 26, 10   02   26, 10.02.026
    // Regex regex_date ("^\\s*(\\d{1,2})([ .])(\\d{1,2})\\2(\\d{2}|\\d{4})\\s*$")

    // Matches dates in the formats: DD.MM.YY or DD MM YY
    // - Day and month: 1 or 2 digits
    // - Year: exactly 2 digits
    // - Separator: either a single dot or a single space
    // - No extra spaces allowed between components (only around the whole date)
    // Valid examples: 10.02.26, 1 9 26
    // Invalid examples: 10. 02. 26, 10   02   26, 10.02.026
    string regex_date_str = ("^\\s*(\\d{1,2})[ .](\\d{1,2})[ .](\\d{2})\\s*$")

    Component _ { // here only to scope the following components
        Regex regex_date (regex_date_str)
        T1.init_text =: T1.text // Initialize time1 by passing T1.init_text to T1.text to ensure T1 is valid when first enabled. 
                                // Since T1.text is normally only assigned on Return key press, it must be explicitly initialized with T1.init_text during setup.
        T1.text =:> regex_date.input

        regex_date.matched.true -> {
            to_time($regex_date.[3], $regex_date.[2], $regex_date.[1]) =: time1
        }
        regex_date.matched.false -> {
            -1 =: time1
        }
        (time1 != -1) =:> T1_valid

        T1_valid.false -> {
            #FF0000 =: T1.text_color   // [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red
        }

        T1_valid.true -> {
            #000000 =: T1.text_color   // ...else...
        }
    }

    Switch T2_sw(false) {
        Component false {
            T1_valid =:> B_valid
        }
        Component true {
            
            
            Regex regex_date_2 (regex_date_str)
            T2.init_text =: T2.text // Initialize time2 by passing T2.init_text to T2.text to ensure T2 is valid when first enabled. 
                                    // Since T2.text is normally only assigned on Return key press, it must be explicitly initialized with T2.init_text during setup.
            T2.text =:> regex_date_2.input
            regex_date_2.matched.true -> {
                to_time($regex_date_2.[3], $regex_date_2.[2], $regex_date_2.[1]) =: time2
            }
            regex_date_2.matched.false -> {
                -1 =: time2
            }
            time2 != -1 =:> T2_valid

            T2_valid.false -> {
                #FF0000 =: T2.text_color    // [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red
            }
            
            T2_valid.true -> {
                #000000 =: T2.text_color    // ...else...
            }

            T1_valid && T2_valid && (time1 <= time2) =:> B_valid // [7GUIs] [When C has the value “return flight”] and T2’s date is strictly before T1’s then B is disabled.
        }
    }

    is_return_flight =:> T2_sw.state  // [7GUIs] When C has the value “return flight”...

    B_valid.true  -> B.enable
    B_valid.false -> B.disable
    // "--\nT1_valid " + T1_valid + "\nT2_valid " + T2_valid + "\ntime1 " + time1 + "\ntime2 " + time2 + "\n(time1 < time2) " + to_string((time1 < time2)) + "\nB_valid " + B_valid =:> tp.input
}


// original text
// [7GUIs] The task is to build a frame containing a combobox C with the two options “one-way flight” and “return flight”, 
// [7GUIs] two textfields T1 and T2 representing the start and return date, respectively, 
// [7GUIs] and a button B for submitting the selected flight. 
// [7GUIs] T2 is enabled iff C’s value is “return flight”. 
// [7GUIs] When C has the value “return flight” and T2’s date is strictly before T1’s then B is disabled. 
// [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red and B is disabled. 
// [7GUIs] When clicking B a message is displayed informing the user of his selection (e.g. “You have booked a one-way flight on 04.04.2014.”). 
// [7GUIs] Initially, C has the value “one-way flight” and T1 as well as T2 have the same (arbitrary) date (it is implied that T2 is disabled).

// [7GUIs] The focus of Flight Booker lies on modelling constraints between widgets on the one hand
// [7GUIs] and modelling constraints within a widget on the other hand.
// [7GUIs] Such constraints are very common in everyday interactions with GUI applications.

// [7GUIs] A good solution for Flight Booker will make the constraints clear, succinct and explicit in the source code and not hidden behind a lot of scaffolding.

// [7GUIs] Flight Booker is directly inspired by the Flight Booking Java example in Sodium with the simplification of using textfields for date input instead of specialized date picking widgets as the focus of Flight Booker is not on specialized/custom widgets.

