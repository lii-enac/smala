use core
use base

_define_
DoubleClick (Component picking, Component frame)
{
	Double squareDist (0)
    Double distThreshold (4)
    Int nbClick (2)
    Incr count (1)
    resetCount = 0 =: count.state : 1
    Bool reachedNbClick (0)
      
    Bool draggedAboveDist (0)
    (squareDist > distThreshold) => draggedAboveDist

	Component clicked
	Component doubleClicked

    Double off_x (0)
    Double off_y (0)
    (frame.move.x - off_x) * (frame.move.x - off_x) 
        + (frame.move.y - off_y) * (frame.move.y - off_y)
        => squareDist

	FSM fsm {
		State idle
		State pressed {
			frame.move.x =: off_x
            frame.move.y =: off_y
		}
		idle -> pressed (picking.press)
        pressed -> idle (draggedAboveDist.true, resetCount)
        pressed -> idle (picking.release, clicked)
	}

	clicked -> count
    (count.state >= nbClick) => reachedNbClick
    reachedNbClick.true -> doubleClicked
    doubleClicked -> resetCount
}