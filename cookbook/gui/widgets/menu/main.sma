use core
use base
use display
use gui

//import Menu
import gui.widgets.DropDownMenu
_main_
Component root
{
  Frame f ("my frame", 0, 0, 500, 500)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1

  NoOutline _()
  FillColor _ (40, 41, 35)
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.width =:> bkg.width
  f.height =:> bkg.height
  


  DropDownMenu menu (100, 10)
  addChildrenTo menu.choices {
    String _ ("Choice 1")
    String _ ("Second choice")
    String _ ("Another choice")
    String _ ("Finally")
  }
  FillColor _ (210, 210, 210)
  FontSize _ (4, 20)
  Text choice (250, 26, "Choice 1")
  menu.selected =:> choice.text
}
