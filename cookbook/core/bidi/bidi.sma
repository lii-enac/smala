/*
*  Smala cookbook bidi
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2019-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Stéphane Conversy <stephane.conversy@enac.fr>
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

import HMI

// does not work yet without _AUTHORIZE_CYCLE, but it would be excellent it if would
// idea: when detecting a cycle, test if it's with properties
// if yes
// test previous value: if it's the same, silently ignore the cycle
// if it's not the same, raise the error
// idea2: when detecting a cycle, do nothing, just don't activate again
// idea3: reuse cycle number/id

// could have been:
// f2c : (5./9.) * (fahrenheit-32) => celsius
// c2f: inverse (f2c)
// could be: fahrenheit <=> hmi.sf.value

_main_
Component root {
  _AUTHORIZE_CYCLE = 1

  // 1st bidi: fahrenheit <=> celsius model
  Double fahrenheit (0)
  Double celsius (0)

  (5./9.) * (fahrenheit - 32) => celsius
  (9./5.  *  celsius)   + 32  => fahrenheit

  // 2nd bidi: slider position <=> value in sliders of the HMI (value is actually an VM from MVVM )
  HMI hmi()

  // 3rd bidi: M <=> VM
  fahrenheit => hmi.sf.value
  hmi.sf.value => fahrenheit
  
  celsius => hmi.sc.value
  hmi.sc.value => celsius
}
