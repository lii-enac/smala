use core
use base
use display
use gui

_define_
Bubble (string _label, double _y, int sent, Process container) {
  Translation global_position (0, _y)
  x aka global_position.tx
  y aka global_position.ty

  String msg (_label)

  svg = load_from_XML ("img/bubble.svg")
  bg << svg.layer1.bubble.main_rect
  content << svg.layer1.bubble.content
  msg =: content.text
  Component timestamp {
    FillColor _ (#595959)
    time << svg.layer1.time
    bg.x + bg.width - 25 =:> time.x
    bg.y + bg.height - 4 =:> time.y
    WallClock wc // Probably to much here a simple function call should be enough
    "%H:%M" =: wc.format
    wc.state_text =: time.text
  }
  Component corner {
    if (sent == 1) {
      14 =: content.x
      //left << svg.layer1.bubble.left_arrow
    } else {
      Translation pos_right (0, 0)
      bg.width + 2 =:> pos_right.tx
      // right << svg.layer1.bubble.right_arrow
      #AD8089 =: bg.fill.value //, right.fill.value
      container.width - bg.width - 14 =:> global_position.tx
    }
  }
  Double width (0)
  Double height (0)
  bg.height =:> height
  bg.width + 14 =:> width
  content.width + timestamp.time.width + 25 =:> bg.width
}
