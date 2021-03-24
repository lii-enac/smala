/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2021)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Poirier <mathieu.poirier@enac.fr>
 *
 */

use core
use base
use display
use gui

 _main_
 Component root
 {
  Frame f ("fill and delete content", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  FillColor _ (100, 50, 50)
  Rectangle background (0, 0, 600, 400)

  Layer drawing_layer {
    Component canvas
  }

  FillColor _ (0, 0, 255)
  Rectangle fill_rectangle_button (150, 450, 100, 100)

  FillColor _ (0, 255, 0)
  Circle fill_circle_button (450, 500, 50)

  fill_rectangle_button.press -> fill_rectangle : (root) {
    // clean layer
    delete_content root.drawing_layer.canvas
    // refill layer
    addChildrenTo root.drawing_layer.canvas {
      FillColor _ (#0000FF)
      Rectangle r1 (100, 100, 40, 40)
      Rectangle r2 (150, 100, 40, 40)
      Rectangle r3 (200, 100, 40, 40)
    }
  }

  fill_circle_button.press -> fill_circle : (root) {
    // clean layer
    delete_content root.drawing_layer.canvas
    // refill layer
    addChildrenTo root.drawing_layer.canvas {
      FillColor _ (#00FF00)
      Circle c1 (100, 100, 20)
      Circle c2 (150, 100, 20)
      Circle c3 (200, 100, 20)
    }
  }
}

