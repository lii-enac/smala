/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2025)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *    Vincent Peyruqueou  <vincent.peyruqueou@enac.fr>
 *
 */

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
import gui.widgets.StandAloneComboBox


_main_
Component root {

  Frame f ("previous", 0, 0, 650, 550)
  f.background_color.r = 127
	f.background_color.g = 127
	f.background_color.b = 127
  
  Exit ex (0, 1)
  f.close -> ex

  mouseTracking = 1

  TextPrinter tp

  // string default_type = "Double"
  // String type (default_type)
  // "Selected type = '" + type + "'" =:> tp.input

  Bool b1 (false)
  Bool b2 (true)

  Int i1 (20)
  Int i2 (144)

  // Double d1 (0.001)
  Double d1 (0.005)
  Double d2 (3.1416)
  // Double d3 (3.1415926535)
  // Double d4 (3.1415926536)

  String s1 ("Hello")
  String s2 ("World !")

  PreviousBool prev_bool (false)
  // PreviousBool prev_bool (false, false)
  PreviousBool prev_bool_diff (false, true) // the output property (previous value) is set only if the new value (input property) is different

  PreviousInt prev_int (0)
  // PreviousInt prev_int (0, false)
  PreviousInt prev_int_diff (0, true) // the output property (previous value) is set only if the new value (input property) is different
  
  PreviousDouble prev_dbl (0.0)
  // PreviousDouble prev_dbl (0.0, false)
  PreviousDouble prev_dbl_diff (0.0, true)  // the output property (previous value) is set only if the new value (input property) is different
  
  PreviousText prev_str ("")
  // PreviousText prev_str ("", false)
  PreviousText prev_str_diff ("", true) // the output property (previous value) is set only if the new value (input property) is different

  Component upper_layer

  StandAlonePushButton btn1 ("set 1st value", 150, 10)
  StandAlonePushButton btn2 ("set 2nd value", 150, 40)

  // StandAloneComboBox combo_box (upper_layer, default_type, 300, 20)
  // addChildrenTo combo_box.str_items {
  //   String _ ("Bool")
  //   String _ ("Int")
  //   String _ ("Double")
  //   String _ ("String")
  // }

  // combo_box.selected_item => type

  btn1.click -> {
    i1 =: prev_int.input
    i1 =: prev_int_diff.input

    d1 =: prev_dbl.input
    d1 =: prev_dbl_diff.input

    s1 =: prev_str.input
    s1 =: prev_str_diff.input

    b1 =: prev_bool.input
    b1 =: prev_bool_diff.input
  }

  btn2.click -> {
    i2 =: prev_int.input
    i2 =: prev_int_diff.input

    d2 =: prev_dbl.input
    d2 =: prev_dbl_diff.input

    s2 =: prev_str.input
    s2 =: prev_str_diff.input

    b2 =: prev_bool.input
    b2 =: prev_bool_diff.input
  }

  Component labels {
    FillColor _ (#000000)
    Text lbl_int (5, 170, "int")
    Text lbl_dbl (5, 270, "dbl")
    Text lbl_str (5, 370, "str")
    Text lbl_bool (5, 470, "bool")
  }

  FontWeight _ (DJN_BOLD)
  FontSize _ (5, 16) // 5 = pixel

  FillColor _ (#333333)
  Text lbl_previous (50, 100, "PREVIOUS")
  Text txt_prev_int (50, 150, "")
  Text txt_prev_diff_int (50, 190, "")
  Text txt_prev_dbl (50, 250, "")
  Text txt_prev_diff_dbl (50, 290, "")
  Text txt_prev_str (50, 350, "")
  Text txt_prev_diff_str (50, 390, "")
  Text txt_prev_bool (50, 450, "")
  Text txt_prev_diff_bool (50, 490, "")

  FillColor _ (#AADDAA)
  Text lbl_new (150, 100, "NEW")
  Text txt_new_int (150, 150, "")
  Text txt_new_diff_int (150, 190, "")
  Text txt_new_dbl (150, 250, "")
  Text txt_new_diff_dbl (150, 290, "")
  Text txt_new_str (150, 350, "")
  Text txt_new_diff_str (150, 390, "")
  Text txt_new_bool (150, 450, "")
  Text txt_new_diff_bool (150, 490, "")
  
  FillColor _ (#000000)
  Text _ (250, 95, "The output property (previous value) is set")
  Text _ (250, 115, "only if the new value (input property) is different")
  Text _ (250, 190, "<-- DIFF ONLY")
  Text _ (250, 290, "<-- DIFF ONLY")
  Text _ (250, 390, "<-- DIFF ONLY")
  Text _ (250, 490, "<-- DIFF ONLY")

  prev_int.input =:> txt_new_int.text         // NEW value
  prev_int.output =:> txt_prev_int.text       // PREVIOUS value
  prev_int_diff.input =:> txt_new_diff_int.text   // NEW value
  prev_int_diff.output =:> txt_prev_diff_int.text // PREVIOUS value

  prev_dbl.input =:> txt_new_dbl.text         // NEW value
  prev_dbl.output =:> txt_prev_dbl.text       // PREVIOUS value
  prev_dbl_diff.input =:> txt_new_diff_dbl.text   // NEW value
  prev_dbl_diff.output =:> txt_prev_diff_dbl.text // PREVIOUS value

  prev_str.input =:> txt_new_str.text         // NEW value
  prev_str.output =:> txt_prev_str.text       // PREVIOUS value
  prev_str_diff.input =:> txt_new_diff_str.text   // NEW value
  prev_str_diff.output =:> txt_prev_diff_str.text // PREVIOUS value

  prev_bool.input =:> txt_new_bool.text         // NEW value
  prev_bool.output =:> txt_prev_bool.text       // PREVIOUS value
  prev_bool_diff.input =:> txt_new_diff_bool.text   // NEW value
  prev_bool_diff.output =:> txt_prev_diff_bool.text // PREVIOUS value


  // For combobox
  //moveChild upper_layer >>

}

