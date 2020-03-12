/*
*  Pinch to Zoom demo app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2019)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Sébastien Leriche <sebastien.leriche@enac.fr>
*    Nicolas Saporito <nicolas.saporito@enac.fr> 
*
*/

use core
use base
use display
use gui


//this action monitors the distance between the 2 touches 
//and binds the resulting scale factor to the root scale factor



_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close -> ex
  Dictionary d_touch
  NoFill _
  OutlineColor _ (100,100,255)
  OutlineWidth _ (10)
  OutlineOpacity _ (0.5)

  Component fixedScene //will contain the visualization of touches

  //component to support the accumulation of transformations (scaling here)
  Homography transforms

  //simple scene
  Rectangle _ (100,100,500,200,5,5)
  Circle _ (500,500,200)

  //touch

  f.touches.$added->(root) {
    t = getRef (&root.f.touches.$added)
    addChildrenTo root.fixedScene {
      Component fingerConnector {
        Circle finger (-100, -100, 100)
        t.move.x =:> finger.cx
        t.move.y =:> finger.cy
      }
    setRef (&root.d_touch.key, &t)
    setRef (&root.d_touch.value, &fingerConnector)
    run root.d_touch.add
    }
  }
  f.touches.$removed->(root) {
    t = getRef (&root.f.touches.$removed)
    setRef (&root.d_touch.key, &t)
    p = getRef (&root.d_touch.value)
    run root.d_touch.delete
    delete p
  }
  TextPrinter print_b
  RefProperty p1 (0)
  RefProperty p2 (0)

  //pinching requires to have exactly 2 touches
  Switch pinchSw (idle) {
    Component idle
    Component pinching {
      Double d (1) // current distance between the 2 touches
      Double p1x (0)
      Double p1y (0)
      Double p2x (0)
      Double p2y (0)

      p1.$value.move.x =:> p1x
      p1.$value.move.y =:> p1y

      p2.$value.move.x =:> p2x
      p2.$value.move.y =:> p2y

      //compute initial distance between p1 and p2, set the result to property d0
      Previous prev (1)
      Sqrt sqrt (0)
      (p2x-p1x)*(p2x-p1x)+(p2y-p1y)*(p2y-p1y) =:> sqrt.input
      sqrt.output =:> d

      //compute and bind the scale factor and center
      (p2x+p1x)/2 =: transforms.leftScaleBy.cx
      (p2y+p1y)/2 =: transforms.leftScaleBy.cy

      d =: prev.input
      d =:> prev.input
      d / prev.output =:> root.transforms.leftScaleBy.sx, root.transforms.leftScaleBy.sy
    }
  }
  Bool test (0)
  RefProperty null_ref (0)
  f.touches.size == 2 => test
  test.true -> (root) {
    setRef (&root.p1, &root.f.touches.1)
    setRef (&root.p2, &root.f.touches.2)
    root.pinchSw.state = "pinching"
  }
  AssignmentSequence set_null (1) {
    null_ref =: p1, p2
    "idle" =: pinchSw.state
  }
  test.false -> set_null
}
