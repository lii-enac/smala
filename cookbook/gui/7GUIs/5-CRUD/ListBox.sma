/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017-2026)
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

// import gui.widgets.AbstractBox
import gui.widgets.VBox


_define_
// ListBox (Process _model) inherits AbstractBox () {
// ListBox (Process _models) inherits VBox () {
ListBox () inherits VBox () {
  // models aka _models

  this.space = 2

  Component bg {
    // FillOpacity op (0.5)
    FillColor fill_c (#FFFFFF)

    OutlineWidth _ (2)
    OutlineColor outln_c (#FF0000)

    Rectangle r (0, 0, 0, 0, 5, 5)
    
    this.preferred_width == -1 ? this.min_width : this.preferred_width =:> r.width
    this.preferred_height == -1 ? this.min_height : this.preferred_height =:> r.height
  }

//   Translation offset (0, 0)
//   Text ui (0, 0, _label)
//   text aka ui.text
//   this.height/2 + ui.ascent/2 - 1 =:> offset.ty

//   ui.width =: this.min_width
//   this.preferred_width == -1 ? ui.width : this.preferred_width =: this.preferred_width
//   ui.height =: this.min_height
//   this.preferred_height == -1 ? ui.height : this.preferred_height =: this.preferred_height
}