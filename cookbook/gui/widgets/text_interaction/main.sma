// Text interaction widgets reference

use core
use base
use display
use gui

import gui.widgets.StandAloneLabel
import gui.widgets.StandAloneSelectableLabel
import gui.widgets.StandAloneUITextField

_main_
Component root {
  Frame f ("Text interaction widgets", 120, 120, 820, 460)
  f.close ->! mainloop

  FillColor bg (#F3F0E8)
  Rectangle background (0, 0, 820, 460)

  FillColor title_color (#202020)
  Text title (24, 34, "Labels and text fields")

  FillColor help_color (#555555)
  Text hint (24, 60, "Selectable rows support drag selection and Ctrl+C.")

  OutlineColor divider_color (#C8C8C8)
  OutlineWidth divider_width (1)
  Line divider (24, 78, 780, 78)

  FillColor label_color (#303030)
  Text l1 (24, 105, "StandAloneLabel")
  Text l2 (24, 145, "StandAloneSelectableLabel")
  Text l3 (24, 195, "StandAloneUITextField")
  Text l4 (24, 235, "StandAloneUITextField read_only")
  Text l5 (24, 275, "StandAloneUITextField custom selection colors")
  Text l6 (24, 315, "StandAloneUITextField disabled")

  StandAloneLabel plain_label (360, 86, 280, 24, "Plain label: light, not selectable")
  plain_label.text_color = #101010
  plain_label.min_width =:> plain_label.width
  plain_label.min_height =:> plain_label.height

  StandAloneSelectableLabel selectable_label (360, 126, 330, 24, "Selectable label: drag me, then Ctrl+C")
  selectable_label.text_color = #101010
  selectable_label.min_width =:> selectable_label.width
  selectable_label.min_height =:> selectable_label.height

  StandAloneUITextField editable (360, 176, 280, 24)
  "Editable text field" =: editable.init_text
  editable.bg_color.value = #FFFFFF

  StandAloneUITextField readonly (360, 216, 280, 24)
  "Read-only text field, selectable" =: readonly.init_text
  readonly.read_only = 1

  StandAloneUITextField custom_selection (360, 256, 280, 24)
  "Custom selection colors" =: custom_selection.init_text
  custom_selection.bg_color.value = #FFFFFF
  custom_selection.selection_color = #FFD166
  custom_selection.text_selected_color = #000000

  StandAloneUITextField disabled (360, 296, 280, 24)
  "Disabled text field" =: disabled.init_text
  |-> disabled.disable
}
