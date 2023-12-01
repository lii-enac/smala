/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017-2023)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */

use core
use base
use display
use gui

_main_
Component root {
  Frame f ("f", 0, 0, 600, 600)
  
  Exit ex (0, 1)
  f.close -> ex
  
  TextPrinter tp

  String reference ("Hello World !")
  String str_selected ("")

  // TextComparator comparator (toString(reference), "")
  
  TextContainer container (toString(reference), "")
  str_selected => container.right


  FillColor _ (255, 255, 255)
  FontSize _ (5, 16)
  Text lbl_title (20, 20, "Click on buttons to test if '" + toString(reference) + "' contains their label ?")

  FontWeight bold (75)
  Text lbl_ref (20, 65, toString(reference))

  FontWeight _ (50)
  Text lbl_contains (130, 65, "contains ?")

  Text lbl_selected (220, 65, "")
  str_selected => lbl_selected.text

  Component feedback {
    FillColor fc (#AAAAAA)
    OutlineColor _ (#CCCCCC)
    OutlineWidth _ (1)
    Circle c (350, 60, 12)
    container.output ? #00FF00 : #FF0000 => fc.value
  }


  // Buttons
  List tests {
    String _ ("Hello World !")
    String _ ("Hello")
    String _ ("Bonjour")
    String _ ("World !")
    String _ ("Wozld !")
    String _ ("ld !")
    String _ ("lo Wo")
    String _ ("lo WO")
    String _ ("")  // Empty string returns FALSE
  }


  int index = 0
  FontWeight _ (50)
  FontSize _ (5, 12)
  TextAnchor _ (DJN_MIDDLE_ANCHOR)

  for str : tests {
    FillColor bg_color (255, 255, 255)
    Rectangle r (20, 100 + index * 50, 120, 40, 5, 5)

    FillColor _ (0, 0, 0)
    Text t (80, 125 + index * 50, toString(str))

    r.press -> {
      #DDDDFF =: bg_color.value
      str =: str_selected
    }

    r.release -> {
      #FFFFFF =: bg_color.value
    }

    index ++
  }

  // Log
  reference + " contains ? '" + str_selected + "' --> " + container.output => tp.input

}

