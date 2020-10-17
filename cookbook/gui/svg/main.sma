use core
use base
use gui

import gui.interactors.PanAndZoom

_main_
Component root {
  Frame frame ("svg", 100, 100, 1000, 500)
  Exit ex (0, 1)
  frame.close -> ex

  // see the screenshots of the illustrator SVG export dialog boxes 
  string path = "res/simple.svg"
  svg = loadFromXML (path)
  //dump svg

  t << svg
}
