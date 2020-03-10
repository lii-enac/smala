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
 *    Nicolas Saporito  <nicolas.saporito@enac.fr>
 *
 */

use core
use base
use display
use gui

import TabManager
import Tab
import Slider


_main_
Component root {
  Frame f ("my frame", 0, 0, 1000, 600)
  Exit ex (0, 1)
  f.close -> ex

  Component bkg {
    NoOutline no
    FillColor fc (108, 109, 105)
    Rectangle r (0, 0, 100, 36, 0, 0)
    FillColor fc2 (40, 40, 40)
    Rectangle r2 (0, 36, 100, 100, 0, 0)
    f.width =:> r.width, r2.width
    f.height - 36 =:> r2.height
  }
  
  Translation pos (0, 5)

  TabManager tabManager
  addChildrenTo tabManager.tabs {
    Tab t1 (f, tabManager, "Tab 1", 1)
    addChildrenTo t1.pane {
      FillColor fc (50, 100, 150)
      Rectangle r (50, 50, 100, 100, 5, 5)
      FillColor fc2 (85, 170, 255)
      Text t (55, 65, "in Tab 1")
    }
    Tab t2 (f, tabManager, "Tab 2", 2)
    addChildrenTo t2.pane {
      FillColor fc (150, 100, 50)
      Rectangle r (55, 100, 100, 100, 5, 5)
      FillColor fc2 (255, 170, 85)
      Text t (60, 115, "in Tab 2")
    }
    Tab t3 (f, tabManager, "Tab 3", 3)
    addChildrenTo t3.pane {
      FillColor fc (10, 150, 50)
      Circle c (155, 100, 30)
      FillColor fc2 (17, 255, 85)
      Text t (131, 104, "in Tab 3")
    }
    Tab t4 (f, tabManager, "Tab 4", 4)
    addChildrenTo t4.pane {
      FillColor fc (150, 50, 100)
      Circle c (155, 150, 30)
      FillColor fc2 (255, 85, 170)
      Text t (131, 154, "in Tab 4")
    }
    Tab t5 (f, tabManager, "Tab 5", 5)
    addChildrenTo t5.pane {
      FillColor fc (150, 50, 10)
      Circle c (300, 150, 30)
      FillColor fc2 (255, 85, 17)
      Text t (276, 154, "in Tab 5")
    }
  }

  Activator select_tab (tabManager.tabs.1.select)
}
