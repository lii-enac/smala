use core
use base
use gui

_define_
TouchMarker (Component _frame) {
	
	frame aka _frame
	Dictionary d_touch

  FillColor _ (100, 100, 255)
  OutlineColor _ (50, 50, 255)
  OutlineWidth _ (2)
  OutlineOpacity _ (0.5)

  frame.touches.$added -> (this) {
    t = getRef (this.frame.touches.$added)
    addChildrenTo this {
      Component fingerConnector {
        Circle finger (-100, -100, 20)
        t.x => finger.cx
        t.y => finger.cy
      }
	    setRef (this.d_touch.key, t)
	    setRef (this.d_touch.value, fingerConnector)
	    run this.d_touch.add
    }
  }
  frame.touches.$removed -> (this) {
    t = getRef (this.frame.touches.$removed)
    setRef (this.d_touch.key, t)
    p = getRef (this.d_touch.value)
    run this.d_touch.delete
    delete p
  }

}