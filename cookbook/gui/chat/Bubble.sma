use core
use base
use display
use gui

_define_
Bubble (string _label, double _x, double _y, int sent) {
  Translation offset (_x, _y)
  x aka offset.tx
  y aka offset.ty

  String msg (_label)
  svg = load_from_XML ("img/bubble.svg")
  FillColor _(Black)
  time << svg.layer1.time
  WallClock wc // Probably to much here a simple function call should be enough
  "%H:%M" =: wc.format
  wc.state_text =: time.text
  bg << svg.layer1.bubble.main_rect
  content << svg.layer1.bubble.content
  msg =: content.text
  if (sent == 1) {
    left << svg.layer1.bubble.left_arrow
  } else {
    Translation pos_right (0, 0)
    bg.width - time.width =:> time.x
    bg.width + 2 =:> pos_right.tx
    right << svg.layer1.bubble.right_arrow
    #AD8089 =: bg.fill.value, right.fill.value
  }
  Double width (0)
  Double height (0)
  bg.y + bg.height =:> height
  bg.x + bg.width =:> width
  content.width + 20 =:> bg.width
}
