/*
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	All rights reserved.
 *
 *
 *	Contributors:
 *		Vinot Jean-Luc <jean-luc.vinot@enac.fr>
 *
 */

use core
use base
use gui
//use input


//imports
import WinSizeAdapt
import SystemTime
import Checkbox
import gui.widgets.Button

_main_
Component root {

	Frame f ("Smala Clock by J.-L. Vinot", 0, 0, 600, 700)
	Exit ex (0, 1)
	f.close -> ex
	gui = loadFromXML ("img/Clock5.svg")
		
	/* create and adapt a background shape */
	back << gui.horloge.back
	f.width => back.width
	f.height => back.height

	// create rotation angles for the 3 needles
	Double sa (0.0)  // rotation angle of second needle
	Double ma (0.0)  // rotation angle of minute needle
	Double ha (0.0)  // rotation angle of hour needle

	// create SystemTime Component
	SystemTime st (f)

	// connect SystemTime sec, min and hour values to rotation angles of needles
	st.sec * 6 => sa
	(st.min * 6) + (st.sec / 10) => ma
	(st.hour * 30) + (st.min / 2) + (st.sec / 120) => ha


	Component scene {

		/* create pan & zoom transform elements to adapt scene transforms to window resizing */
		Translation trScene (0, 0)
		Scaling scScene (1.0, 1.0, 0, 0)

		/* adapt the trame content to the window resizing*/
		WinSizeAdapt wsa (f, trScene, scScene)
  		
	
		/* create a view component in the graphscene to apply WinSizeAdapt pan and zomm transforms on graphical contents  */
		Component view {

			/* create Clock dial SVG component (from SVG file) */
			dial << gui.horloge.dial

			/* Connect the rotation angles to the svg hands */
			sa => dial.needles.seconds_shadow_translation.seconds_shadow.rotate.a, dial.needles.seconds.rotate.a
			ma => dial.needles.minutes_shadow_translation.minutes_shadow.rotate.a, dial.needles.minutes.rotate.a
			ha => dial.needles.hours_shadow_translation.hours_shadow.rotate.a, dial.needles.hours.rotate.a

			/* create a Chechbox to select diverse time mode */
			Checkbox cb (f, 5, 60, 620)

			// define labels of the checkbox
  			cb.entries.[3].label = "play"
  			cb.entries.[2].label = "pause"
  			cb.entries.[1].label = "rewind"
  			cb.entries.[4].label = "forward"
  			cb.entries.[5].label = "speedy"

  			// connect Chechbox selection to SystemTime modes
  			cb.choice => st.timemode.state


		}

	}
	scene.view.cb.entries.[3].fsm.initial = "st_selected"
	Button b (f, "Init time", 500, 600)
	b.click -> st.compute
	f.width - 100 =:> b.x
	f.height - 100 =:> b.y
}
