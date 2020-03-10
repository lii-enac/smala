/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Nicolas Saporito  <nicolas.saporito@enac.fr>
*
*/

use core
use base
use display
use gui

import SimpleZoom
import MouseCenteredZoom

_main_
Component root {

  Frame frame ("Zoom", 0, 0, 800, 800)
  Exit ex (0, 1)
  frame.close -> ex
  mouseTracking = 1

  // Transforms
  Scaling scaling (1, 1, 0, 0)
  Translation translation (0, 0)

  // Simple zoom around frame center
  SimpleZoom zoom (frame, scaling)

  // Mouse-centered zoom
  //MouseCenteredZoom zoom (frame, scaling, translation)

  // Graphics
  FillColor _ (70, 70, 70)
  Text ts (200, 200, "The scene can be zoomed by mouse wheel")

  FillColor _ (200, 200, 200)
  FillOpacity _ (0.5)
  OutlineColor _ (70, 70, 70)
  Rectangle rectangle (100, 100, 100, 100, 0, 0)
}

