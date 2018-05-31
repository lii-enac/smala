/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
 use core
 use base
 use gui


  _define_
  TabManager () {
	Spike unselectAll
    Double selected_color (40)
    Double unselected_color (65)
    Double selected_font_color (255)
    Double unselected_font_color (150)
    Double unselected_border_color (55)
    Double selected_border_color (30)
    List tabs
}
