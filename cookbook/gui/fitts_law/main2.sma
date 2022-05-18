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
	// TODO
	// guiard's FxS (Form x Scale) or ID x W with fixed A.
	/*struct condition {
		int F, S;
	};*/

	// traditional (confounded) AxW with Expansion
	struct condition {
		int A, W; // Amplitude, Width
		double E; // Expansion
	};

	std::vector<int> As = {128, 256, 512, 1024};
	std::vector<int> Ws = {8, 16, 32, 64};
	//std::vector<double> Es = {1.}; // Fitts
	//std::vector<double> Es = {2.}; // R&B
	std::vector<double> Es = {.5, 1., 2.}; // Z,C,G&BL

	std::vector<condition> conditions;
	for (auto a: As) {
		for (auto w: Ws) {
			for (auto e: Es) {
				conditions.push_back(condition{a,w,e});
			}
		}
	}

	int num_repeat = 2;
	std::vector<condition> repeats;
	for (int i=0; i< num_repeat; ++i) repeats.insert(repeats.end(), conditions.begin(), conditions.end());
	
	std::random_device rng;
	std::mt19937 urng(rng());
	std::shuffle (repeats.begin(), repeats.end(), urng);
	
	#define GET_PROC(type, varname, parent) auto *varname = dynamic_cast<type*>(parent->find_child (#varname)); assert(varname);

	GET_PROC(SwitchList, measures, root);
	int i=0;
	for (auto * p: measures->children ()) {
		auto *measure = dynamic_cast<Component*>(p); assert(measure);
		GET_PROC(AbstractIntProperty, tw, measure);
		GET_PROC(AbstractIntProperty, td, measure);
		GET_PROC(AbstractDoubleProperty, te, measure);
		tw->set_value (repeats[i].W, true);
		td->set_value (repeats[i].A, true);
		te->set_value (repeats[i].E, true);
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

	// typical 1D Fitts task
  	FittsTask1D fitts(f)
	

	// measurements control

    Int target_width(0)
    Int target_distance(0)
	Double target_expansion(1.0)

	// create a list of measurements
	SwitchList measures {
		// 32+1, +1 to be notified when end is reached FIXME iterator
		for (int i=0; i<32+1; i++) {
			Component measure {
				Int tw(0)
				Int td(0)
				Double te(1.0)
				tw =: target_width
				td =: target_distance
				te =: target_expansion
			}
		}
	}

	// populate measure properties (tw,td,s) with experimental plans
	cpp_action (&root)

	// after each measurement, go to next measurement
	fitts.ended -> measures.next
	// exit when finished
	(measures.index == 32+1) -> ex


	// condition control

	// distance is not transformed, for all conditions
	target_distance =:> fitts.target_distance

    // Classical Fitts experiment
    // target_width =:> fitts.target_width

  	// MacGuffin & Balakrishnan, 2002 and Zhai, Conversy, Guiard & Beaudouin-Lafon, 2003
	// also works with classical Fitts experiment since target_expansion is 1...
    f.move.x > 0.9 * target_distance
  		? target_width * target_expansion
  		: target_width
  		  =:> fitts.target_width
  	

    // log results

	// hit or miss
	String hit_or_miss ("")
	AssignmentSequence hm_as(1) {
		fitts.control.state =: hit_or_miss
	}
	fitts.control.hit -> hm_as
	fitts.control.miss -> hm_as

	TextPrinter tp

	// header
	"measure\tW\tA\tE\thitmiss\tt" =: tp.input

	// after each try, log the result
	fitts.ended -> {
    	measures.index + "\t" + target_width + "\t" + target_distance + "\t"
		+ target_expansion + "\t"
		+ hit_or_miss + "\t" + fitts.target_time_acquisition =: tp.input
	}
}

