/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
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
 use display
 use gui

 _define_
 TextLineEdit (Process f, string default_text, double x, double y) {
  String _default_text (default_text)
  String validated_text ("")
  Translation pos (x, y)

  FillColor _ (White)
  Rectangle bkg (0, 0, 190, 21, 3, 3)
  //RectangleClip clip (0, 0, 175, 18)
  Translation _ (5, 2)
  Translation back_move (0, 0)
  FillColor _ (Black)
  TextField field (0, 0, 180, 18, default_text, 1)
  f.key\-pressed =:> field.key_pressed
  f.key\-released =:> field.key_released
  f.key\-pressed_text =:> field.string_input
  
  /* Cursor */
  OutlineColor _ (Black)
  OutlineWidth _ (2)
  Line cursor (0, 0, 0, 15)
  field.cursor_end_x =:> cursor.x1, cursor.x2
  field.cursor_height =:> cursor.y2
  

  AssignmentSequence set_text (1) {
    field.content.text =: validated_text
  }
  field.validate->set_text
  set_text->field.clear  
}
