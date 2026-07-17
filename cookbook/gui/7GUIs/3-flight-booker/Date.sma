/*
*  Smala cookbook 3-flight-booker
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Stephane Conversy <Stephane.conversy@enac.fr>
*
*/
use core
use base
use display
use gui

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

_define_
Date(Process T, Process T_valid, Process time) {
    string regex_date_str = ("^\\s*(\\d{1,2})[ .](\\d{1,2})[ .](\\d{2})\\s*$")
    Regex regex_date (regex_date_str)

    T.init_text =: T.text       // Since T.text is normally only assigned on Return key press, it must be explicitly initialized with T.init_text during setup.
    T.text =:> regex_date.input // Initialize time by passing T.init_text to T.text to ensure T is valid when first enabled.

    regex_date.matched.true -> {
        to_time($regex_date.[3], $regex_date.[2], $regex_date.[1]) =: time
    }
    regex_date.matched.false -> {
        -1 =: time
    }
    (time != -1) =:> T_valid

    T_valid.false -> {
        #FF0000 =: T.bg_color.value   // [7GUIs] When a non-disabled textfield T has an ill-formatted date then T is colored red
    }

    T_valid.true -> {
        #FFFFFF =: T.bg_color.value   // ...else...
    }
}