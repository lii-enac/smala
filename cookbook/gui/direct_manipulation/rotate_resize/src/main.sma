/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Nicolas Saporito <nicolas.saporito@enac.fr>
 *
 */

use core
use base
use display
use gui

import RotateResizeRectangle

_main_
Component root {
	Frame f ("Rotate Resize with Homography", 0, 0, 800, 800)
	Exit ex (0, 1)
	f.close -> ex
	mouseTracking = 1

	FillColor fc (255, 0, 0)
	Text t  (100, 100, "BE CAREFUL !")
	Text t1 (100, 120, "This recipe is not working correctly for now.")
	Text t2 (100, 140, "It has to be remade")


	// A few arbitrary transforms
	// (RotateResizeRectangle below uses local coordinates, so one can transform
	// the scene before it at will without impacting its interactions)
	Rotation arbitraryRotation (22.5, 0, 0)
	Translation arbitraryTranslation (56, 24)
	Scaling arbitraryScaling (1.2, 1.2, 0, 0)
	
	// Graphic styles
	FillColor _ (200, 200, 200)
    OutlineColor _ (70, 70, 70)

    // Scene
	RotateResizeRectangle r1 (f, 150, 200)
	RotateResizeRectangle r2 (f, 350, 100)
}
