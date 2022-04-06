use core
use base
use gui

_define_
StandAloneSlider (Process f, int _x, int _y, int _width, int _lower, int _upper, double _init_val, string _unit)
{
  Translation pos (_x, _y)
  x aka pos.tx
  y aka pos.ty
  Double width (_width)
  Double height (0)
  BoundedValue bv (_lower, _upper, _init_val)
  output aka bv.result
  Double value (_init_val)
  String unit (_unit)

  Component ui {
    Component bg {
      FillColor fc (#303030)
      NoOutline _
      Rectangle bg (0,0, _width, 48, 5, 5)
      width aka bg.width
      height aka bg.height
    }
    Component bg_line {
      FillColor fill (#ffffff)
      NoOutline _
      Rectangle bg_line (24, 19.8808, _width - 48, 8, 4, 4)
      width aka bg_line.width
      height aka bg_line.height
    }
    Component fg_line {
      FillColor fill (#ffa700)
      NoOutline _
      Rectangle bg_line (24, 19.8808, 40, 8, 4, 4)
      width aka bg_line.width
      height aka bg_line.height
    }
    Component halo {
      FillColor fill (#ffa700)
      FillOpacity _ (0.5)
      NoOutline _
      Circle halo (24, 23.8808, 20)
    }
    Component button {
      FillColor fill (#ffa700)
      NoOutline _
      Circle button (24, 23.8808, 10)
      press aka button.press
      move aka button.move
      release aka button.release
      enter aka button.enter
      leave aka button.leave
    }
    Component property {
      FillColor fill (#ffffff)
      FontSize fs (4, 12)
      FontFamily ff ("B612")
      FontWeight _ (75)
      TextAnchor _ (1)
      Text property (23.572605, 27.386652, "100%")
      text aka property.text
    }
  }

  background_color aka ui.bg.fc
  background_line_color aka ui.bg_line.fill
  foreground_line_color aka ui.fg_line.fill
  halo_color aka ui.halo.fill
  button_color aka ui.button.fill
  text_color aka ui.property.fill

  main_bg << ui.bg
  bg << ui.bg_line
  fg << ui.fg_line
  main_bg.width =: width
  main_bg.height =: height
  fg.width = $bg.width

  Translation button_pos ($bg.width, 0)
  Switch halo (idle) {
  	Component idle
  	Component moving {
  	  halo << ui.halo
  	}
  }

  BoundedValue pos_bv (0, $bg.width, $bg.width)
  Int i_output ($bg.width)
  output =:> i_output
  button << ui.button
  prop << ui.property
  Double coeff (1)
  bg.width/ (bg.width-20) =:> coeff
  ((value / (_upper - _lower)) * bg.width + 10*coeff)/coeff =: pos_bv.input, button_pos.tx, fg.width
  value =: bv.input
  FSM button_fsm {
  	State idle {
  		#ffffff =: button.fill.value
      "" =: prop.text
      value->{((value / (_upper - _lower)) * bg.width + 10*coeff)/coeff =: pos_bv.input
              pos_bv.result =: button_pos.tx, fg.width }
  	}
  	State hover {
  		#ffa700 =: button.fill.value
      toString (i_output) + unit =:> prop.text
  	}
  	State moving {
  	  Double offset_x (0)
      Double init_tx (0)
      Double delta_x (0)
      #ffa700 =: button.fill.value
      button_pos.tx =: init_tx
      button.press.x =: offset_x
      f.move.x - offset_x => delta_x
      delta_x + init_tx => pos_bv.input
      pos_bv.result => button_pos.tx, fg.width
      pos_bv.result*((_upper - _lower)/$bg.width) + _lower =:> bv.input
  	  toString (i_output) + unit =:> prop.text
  	}
  	idle->hover (button.enter)
  	hover->idle (button.leave)
  	{hover,idle}->moving (button.press)
  	idle->hover (button.move)
  	moving->idle (button.release)
  }
  button_fsm.state =:> halo.state
}