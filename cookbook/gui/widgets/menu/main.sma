use core
use base
use display
use gui

//import Menu
import gui.widgets.DropDownMenu
_main_
Component root
{
  Frame f ("my frame", 0, 0, 400, 400)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1

  NoOutline _()
  FillColor _ (40, 41, 35)
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.width =:> bkg.width
  f.height =:> bkg.height
  


  DropDownMenu menu (30, 10)
  addChildrenTo menu.choices {
    String _ ("circle")
    String _ ("rectangle")
    String _ ("polyline")
    String _ ("polygon")
  }
  Translation pos (100, 0)
  FillColor _ (210, 210, 210)
  OutlineColor _ (50, 250, 0)
  OutlineWidth _ (3)
  Switch selected (idle) {
    Component idle
    Circle circle (100, 200, 30)
    Rectangle rectangle (70, 170, 60, 60, 2, 2)
    Polyline polyline {
      Point _ (70, 230)
      Point _ (100, 170)
      Point _ (130, 230)
    }
    Polygon polygon {
      Point _ (70, 230)
      Point _ (100, 170)
      Point _ (130, 230)
    }
  }
  TextComparator comp ("", "")
  menu.selected =:> comp.left
  comp.output == 1 ? "idle" : toString(menu.selected) =:> selected.state
}
