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
	// traditional (confounded) AxW
	struct condition {
		int A, W;
		//double scale;
	};

	std::vector<int> As = {128, 256, 512, 1024};
	std::vector<int> Ws = {8, 16, 32, 64};
	std::vector<double> Scales = {.5, 1., 2.};

	std::vector<condition> conditions;
	for (auto a: As) {
		for (auto w: Ws) {
			conditions.push_back(condition{a,w});
			//for (auto s: Scales) {
			//	conditions.push_back(condition{a,w,s});
			//}
		}
	}

	int num_repeat = 2;
	std::vector<condition> repeats;
	for (int i=0; i< num_repeat; ++i) repeats.insert(repeats.end(), conditions.begin(), conditions.end());
	
	std::random_device rng;
	std::mt19937 urng(rng());
	std::shuffle (repeats.begin(), repeats.end(), urng);

	// TODO
	// guiard's FxS (Form x Scale) or ID x W with fixed A.
	/*struct condition {
		int F, S;
	};*/
	
	#define GET_PROC(type, varname, parent) auto *varname = dynamic_cast<type*>(parent->find_child (#varname)); assert(varname);

	GET_PROC(SwitchList, measures, root);
	int i=0;
	for (auto * p: measures->children ()) {
		auto *measure = dynamic_cast<Component*>(p); assert(measure);
		GET_PROC(AbstractIntProperty, tw, measure);
		GET_PROC(AbstractIntProperty, td, measure);
		//GET_PROC(AbstractDoubleProperty, s, measure);
		tw->set_value (repeats[i].W, true);
		td->set_value (repeats[i].A, true);
		//s->set_value (repeats[i].scale, true);
		++i;
		if(i==32) break; // +1 to be notified when end is reached 
	}
}
%}

_main_
Component root
{
	Frame f ("my frame", 0, 0, 1200, 800)
  	Exit ex (0, 1)
  	f.close -> ex
  	mouseTracking = 1

  	FittsTask1D fitts(f)
	
    Int target_width(0)
    Int target_distance(64)
	Double scale(2.0)

    target_distance =:> fitts.target_distance

	SwitchList measures {
		// 32+1, +1 to be notified when end is reached FIXME iterator
		for (int i=0; i<32+1; i++) {
			Component measure {
				Int tw(0)
				Int td(0)
				//Double s(1.0)
				tw =: target_width
				td =: target_distance
				//s =: scale
			}
		}
	}
	cpp_action (&root) // populate list

	fitts.ended -> measures.next
	(measures.index == 32+1) -> ex

    // classic
    target_width =:> fitts.target_width

  	// MacGuffin & Balakrishnan, 2002
    /*
    f.move.x > 0.9 * fitts.target_distance
  		? target_width * scale
  		: target_width
  		  =:> fitts.target_width
  	*/
  	
  	// Zhai, Conversy, Guiard & Beaudouin-Lafon, 2003
    
  	//Double scale(0.5) // FIXME .5 does not work!!
  	//scale(1)
  	//scale(2)
  	// should be in the experimental plan
	/*
	f.move.x > 0.9*fitts.target_distance
  		? target_width * scale
  		: target_width
  	      =:> fitts.target_width
	*/

    // log

	String hit_or_miss ("")
	AssignmentSequence hm_as(1) {
		fitts.control.state =: hit_or_miss
	}
	fitts.control.hit -> hm_as
	fitts.control.miss -> hm_as

	TextPrinter tp
	"measure\tW\tA\tS\thitmiss\tt" =: tp.input // header

	fitts.ended -> {
    	measures.index + "\t" + target_width + "\t" + target_distance + "\t"
		//+ scale + "\t"
		+ hit_or_miss + "\t" + fitts.target_time_acquisition =: tp.input
	}
}

