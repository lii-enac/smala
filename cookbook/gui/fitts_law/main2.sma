use core
use base
use display
use gui
use files

import FittsTask1D

_native_code_
%{
#include <algorithm>
#include <random>
#include <cassert>

void
cpp_action (Process* root)
{
 	//Process *root = reinterpret_cast<Process*> (get_native_user_data (c));

	// traditional, confounded AxW
	struct condition {
		int A, W;
	};

	std::vector<int> As = {128, 256, 512, 1024};
	std::vector<int> Ws = {8, 16, 32, 64};

	std::vector<condition> conditions;
	for(auto a: As) {
		for(auto w: Ws) {
			conditions.push_back(condition{a,w});
		}
	}

	int num_repeat = 2;
	std::vector<condition> repeats;
	for (int i=0; i< num_repeat; ++i) repeats.insert(repeats.end(), conditions.begin(), conditions.end());

	std::random_device rng;
	std::mt19937 urng(rng());
	std::shuffle (repeats.begin(), repeats.end(), urng);

	// guiard's FxS (Form x Scale) or ID x W with fixed A.
	// TODO
	/*struct condition {
		int F, S;
	};*/
	

	auto *measures = dynamic_cast<SwitchList*>(root->find_child ("measures")); assert(measures);
	int i=0;
	for (auto * p: measures->children ()) {
		auto *measure = dynamic_cast<Component*>(p); assert(measure);
		auto *tw = dynamic_cast<AbstractIntProperty*>(measure->find_child ("tw")); assert(tw);
		auto *td = dynamic_cast<AbstractIntProperty*>(measure->find_child ("td")); assert(td);
		tw->set_value (repeats[i].W, true);
		td->set_value (repeats[i].A, true);
		++i;
		if(i==32) break;
	}
}
%}

_main_
Component root
{
	Frame f ("my frame", 0, 0, 800, 800)
  	Exit ex (0, 1)
  	f.close -> ex
  	mouseTracking = 1

  	FittsTask1D fitts(f)
	
    Int target_width(0)
    Int target_distance(64)

    target_distance =:> fitts.target_distance

	SwitchList measures {
		for (int i=0; i<33; i++) {
			Component measure {
				Int tw(0)
				Int td(0)
				tw =: target_width
				td =: target_distance
			}
		}
	}
	cpp_action (&root) // populate list

	fitts.control.init -> measures.next
	(measures.index == 33) -> ex

    // classic
    //target_width =:> fitts.target_width

  	// MacGuffin & Balakrishnan, 2002
    /* 	
    f.move.x > 0.9*fitts.target_distance
  		? target_width*2
  		: target_width
  	=:> fitts.target_width
  	*/
  	
  	// Zhai, Conversy, Guiard & Beaudouin-Lafon, 2003
    /*
  	Double scale(0.5) // FIXME .5 does not work!!
  	//scale(1)
  	//scale(2)
  	// should be random

	  f.move.x > 0.9*fitts.target_distance
  		? target_width*scale
  		: target_width
  	=:> fitts.target_width
  	*/

    // display result

	String hit_or_miss ("")
	AssignmentSequence hm_as(1) {
		fitts.control.state =: hit_or_miss
	}
	fitts.control.hit -> hm_as
	fitts.control.miss -> hm_as

	TextPrinter tp
	fitts.control.init -> {
    	measures.index + " " + hit_or_miss + " " + target_width + " " + target_distance + " " + fitts.target_time_acquisition =: tp.input
	}
}

