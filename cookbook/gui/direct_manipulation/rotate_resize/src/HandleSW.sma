use core
use exec_env
use base
use display
use gui

_define_
HandleSW (Process target, double size) {

    /* ----- Interface ----- */
    Double x (0)
    Double y (0)
    /* --- End Interface --- */

    // Translation to initially place the rectangle
    Translation translation (0, 0)
    x =:> translation.tx
    y =:> translation.ty

    rect aka target.rect
    ref aka target.ref
    transforms aka target.transforms
    minWidth aka target.minWidth
    minHeight aka target.minHeight

    // Make this handle look size-invariant 
    // (apply the inverse of the scalings accumulated on the target)
    Scaling scaling_inv (1, 1, 0, 0)
    1 / transforms.accsx =:> scaling_inv.sx
    1 / transforms.accsy =:> scaling_inv.sy

    // Graphics
    Rectangle handle (0, 0, size, size, 0, 0)
    Double _size (size)
    - _size / 2 =: handle.x, handle.y

    // Store mouse press
    Double x0 (0)
    Double y0 (0)

    FSM resizingFsm {
        State idle

        State pressed {
            // Set up scaling center in rect coord system
            rect.width :: transforms.rightScaleBy.cx
            0 :: transforms.rightScaleBy.cy
            
            // Get mouse press in rect coord system
            ScreenToLocal s2l_mouse (rect)
            handle.press.x =:> s2l_mouse.inX
            handle.press.y =:> s2l_mouse.inY
            s2l_mouse.outX =:> x0
            s2l_mouse.outY =:> y0
        }

        State resizing {
            // Get mouse move in rect coord system
            Double x1 (0)
            Double y1 (0)
            ScreenToLocal s2l_mouse (rect)
            handle.move.x =:> s2l_mouse.inX
            handle.move.y =:> s2l_mouse.inY
            s2l_mouse.outX =:> x1
            s2l_mouse.outY =:> y1

            // Compute and apply scale factors
            Double sx (1)
            1 + (x0 - x1) / rect.width =:> sx
            sx * transforms.accsx * rect.width > minWidth ? sx : 1 =:> transforms.rightScaleBy.sx
            Double sy (1)
            1 + (y1 - y0) / rect.height =:> sy
            sy * transforms.accsy * rect.height > minHeight ? sy : 1 =:> transforms.rightScaleBy.sy
        }

        idle -> pressed (handle.press)
        pressed -> idle (handle.release)
        pressed -> resizing (handle.move)
        resizing -> idle (handle.release)
    }
}
