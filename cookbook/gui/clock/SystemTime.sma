use core
use base
use gui

_native_code_
%{
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
%}

_action_
recordSystemTime (Process comp)
%{
  Process *data = (Process*) get_native_user_data (comp);

  char day_names[7][10] = { "dimanche",
                            "lundi",
                            "mardi",
                            "mercredi",
                            "jeudi",
                            "vendredi",
                            "samedi"
                          };

  char month_names[12][10] = { "janv.",
                               "fev.",
                               "mars",
                               "avr.",
                               "mai",
                               "juin",
                               "juil.",
                               "aout",
                               "sept.",
                               "oct.",
                               "nov.",
                               "dec."
                             };

  /*Store the initial time*/
  time_t current_time = time(NULL);
  struct tm *tm_p;
  tm_p = localtime( &current_time );

  int y = tm_p->tm_year + 1900;
  int mm = tm_p->tm_mon;
  int md = tm_p->tm_mday;
  int wd = tm_p->tm_wday;
  int s = tm_p->tm_sec;
  int m = tm_p->tm_min;
  int h = tm_p->tm_hour;

((IntProperty*)data->find_child ("initsec"))->set_value (s, true);
((IntProperty*)data->find_child ("min"))->set_value (m, true);
((IntProperty*)data->find_child ("hour"))->set_value (h, true);
((IntProperty*)data->find_child ("year"))->set_value (y, true);
((IntProperty*)data->find_child ("mday"))->set_value (md, true);
((TextProperty*)data->find_child ("wday"))->set_value (day_names[wd], true);
((TextProperty*)data->find_child ("month"))->set_value (month_names[mm], true);

  double totalseconds = (double) (h * 3600) + (m * 60) + s;
  ((DoubleProperty*)data->find_child ("initialtime"))->set_value (totalseconds, true);


  printf("SystemTime::recordSystemTime year:%d month:%d monthname:%s day:%d dayname:%s hours:%d minutes:%d seconds:%d total:%f\n", y, mm, month_names[mm], md, day_names[wd], h, m, s, totalseconds);

%}

_define_
SystemTime (Process f) {

  Component tm {
  	Int initsec (0)
    Double sec (0.0)
	  Int year (1900)   // current year
    Int mon (0)       // current month number
    String month ("") // current month string
    Int mday (0)      // current day number in month
    String wday ("")  // current day string
    Int min (0)       // current minute 
    Int hour (0)      // current hour 
    Double initialtime (0.0)
    Double curtime (0.0)
  }

  // interface
  curtime aka tm.curtime
  sec aka tm.sec
  min aka tm.min
  hour aka tm.hour
  year aka tm.year
  monthname aka tm.month
  month aka tm.mon
  dayname aka tm.wday
  day aka tm.mday

  // adjust initial time with system time (compute only at creation)
  NativeAction compute (recordSystemTime, tm, 0)
  compute ~> tm.initsec
  compute ~> tm.min
  compute ~> tm.hour
  compute ~> tm.year
  compute ~> tm.mday
  compute ~> tm.wday
  compute ~> tm.month

  /* create clock (1/50 s) and increment smala components to record time movement */
  Clock cl (50)
  Incr incr (0)
  compute->{ 0 =: incr.state }
  // init time advance ratios for incr.delta value
  Int normal_ratio (1)
  Int reverse_ratio (-5)
  Int forward_ratio (10) 
  Int speedy_ratio (60)

  // connect advance of time incr to time data
  tm.initialtime + (incr.state / 20) => tm.curtime
  tm.curtime / 3600 => tm.hour
  (tm.curtime - (tm.hour * 3600)) / 60 => tm.min
  tm.curtime - ((tm.hour * 3600) + (tm.min * 60)) => tm.sec


  /* create a global switch component with 5 states: play, pause, rewind, forward, speedy
     to change time progress mode according to user selection */
  Switch timemode (play) {

    // play mode: normal progress of time
    Component play {
      normal_ratio =: incr.delta
      cl.tick -> incr
    }

    // pause mode: stop the progress of time
    Component pause {
          
    }

    // rewind mode: inverse the progress of time
    Component rewind {
      reverse_ratio =: incr.delta
      cl.tick -> incr
    }

    // fast time mode : X5 time advance
    Component forward {
      forward_ratio =: incr.delta
      cl.tick -> incr
    }

    // very fasttime mode : X20 time advance
    Component speedy {
      speedy_ratio =: incr.delta
      cl.tick -> incr
    }    

  }

  
}
