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

import TabManager
import Tab
import Slider

_main_
Component root {
  Frame f ("my frame", 0, 0, 800, 600)
  Exit ex (0, 1)
  f.close -> ex

  Component bkg {
    NoOutline no
    FillColor fc (108, 109, 105)
    Rectangle r (0, 0, 100, 36, 0, 0)
    FillColor fc2 (40, 40, 40)
    Rectangle r2 (0, 36, 100, 100, 0, 0)
    f.width => r.width, r2.width
    f.height - 36 => r2.height
  }
  Translation pos (0, 5)

  TabManager tabManager
  addChildrenTo tabManager.tabs {
    Tab t1 (f, tabManager, "Tab 1", 0)
      addChildrenTo t1.pane {
      FillColor fc (50, 100, 150)
      Rectangle r (50, 50, 100, 100, 5, 5)
    }
    Tab t2 (f, tabManager, "Tab 2", 1)
    addChildrenTo t2.pane {
      FillColor fc (150, 100, 50)
      Rectangle r (50, 100, 100, 100, 5, 5)
    }
    Tab t3 (f, tabManager, "Tab 3", 2)
    addChildrenTo tabManager.tabs.3.pane {
      FillColor fc (10, 150, 50)
      Circle c (150, 100, 30)
    }
  }
  Activator set_top (tabManager.tabs.3.setOnTop)
}

run root
run syshook
