/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2021-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

import Item

 _main_
 Component root
 {
  Frame f ("fill and delete content", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  // mouseTracking = 1

  Int number_of_item (500)

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
      Component component1 {
        for (int i = 0 ; i < root.number_of_item ; i++) {
          Process item = Item (root.drawing_layer.canvas, "item"+ to_string(i), #0000FF, 100+(20*i), 100)
        }
      }

      Component component2 {
        for (int i = 0 ; i < root.number_of_item ; i++) {
          Process item = Item (root.drawing_layer.canvas, "item"+ to_string(i), #00FF00, 100+(20*i), 200)
        }
      }
    }
  }

  fill_circle_button.press -> fill_circle : (root) {
    // clean layer
    delete_content root.drawing_layer.canvas
    // refill layer
    addChildrenTo root.drawing_layer.canvas {
      Component component1 {
        for (int i = 0 ; i < root.number_of_item ; i++) {
          Process item = Item (root.drawing_layer.canvas, "item"+ to_string(i), #eaff00, 100+(20*i), 100)
        }
      }

      Component component2 {
        for (int i = 0 ; i < root.number_of_item ; i++) {
          Process item = Item (root.drawing_layer.canvas, "item"+ to_string(i), #e100ff, 100+(20*i), 200)
        }
      }
    }
  }
}

