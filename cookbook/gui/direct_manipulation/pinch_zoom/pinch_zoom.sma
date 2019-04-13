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
use gui
use display

_native_code_
%{
  #include <map>
  #include <cmath>

  //map to store the various fingerConnector components  
  std::map<Process*, Process*> fcMap;

  //store in the map
  Process* addTouch (Process* root, Process* added, Process* fingerConnector) {
    fcMap.insert(std::pair<Process*, Process*> (added, fingerConnector));
    ((IntProperty*) root->find_component ("nbTouches"))->set_value ((int) fcMap.size(), 1);
  }

  //find in the map which fingerConnector has to be removed
  Process* findTouch (Process* root, Process* removed) {
    std::map<Process*, Process*>::iterator it;
    
    it = fcMap.find(removed);
    Process* found;
    if (it != fcMap.end()) {
      found = it->second; 
      fcMap.erase(it);
    }

    ((IntProperty*) root->find_component ("nbTouches"))->set_value ((int) fcMap.size(), 1);
    return found;
  }

  //n is 0 or 1
  Process * getTouch (int n) {
  	std::map<Process*, Process*>::iterator it;
  	it = fcMap.begin();
  	if (n>0) {
  		++it;
  	}
  	return it->first;
  }

%}

//this action monitors the distance between the 2 touches 
//and binds the resulting scale factor to the root scale factor
_action_
pinch_action (Component src, Component root)
{
	Process p1 = CCall(getTouch,0)
	Process p2 = CCall(getTouch,1)
	addChildrenTo root {
		Component pinchZoomConnector {
      Double d0 (1) //initial distance between the 2 touches
      Double d (1) // current distance between the 2 touches

      //compute initial distance between p1 and p2, set the result to property d0
      Sqrt sqrt0 (0)      
      (p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y) =: sqrt0.input
      sqrt0.output => d0

      //current distance p1->p2
      Sqrt sqrt (0)      
			(p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y) => sqrt.input
			sqrt.output => d

			//compute and bind the scale factor and center
      (p2.x+p1.x)/2 =: root.transforms.leftScaleBy.cx
      (p2.y+p1.y)/2 =: root.transforms.leftScaleBy.cy
      
      //I would have been delighted to be able to write d/(last(d)) => xxx.sx, but...
      AssignmentSequence as (1) {
        d/d0 =: root.transforms.leftScaleBy.sx, root.transforms.leftScaleBy.sy
        d =: d0
      }
      d -> as

		}
	}
}

_action_
delete_action (Component src, Component root)
{
	delete root.pinchZoomConnector
}

_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)
  
  NoFill _
  OutlineColor _ (100,100,255)
  OutlineWidth _ (10)
  OutlineOpacity _ (0.5)

  Component fixedScene {
    //will contain the visualization of touches
  }

  //component to support the accumulation of transformations (scaling here)
  Homography transforms

  //simple scene
  Rectangle _ (100,100,500,200,5,5)
  Circle _ (500,500,200)

  
  //touch
  Int nbTouches (0)			//exactly 2 touches are required
  f.touches.$added-> (root) {
    t = getRef (root.f.touches.$added)
    addChildrenTo root.fixedScene {
      Component fingerConnector {
        Circle finger (-100, -100, 100)
        t.x => root.fixedScene.fingerConnector.finger.cx
        t.y => root.fixedScene.fingerConnector.finger.cy
      }
    }
    Process _ = CCall (addTouch, root, t, root.fixedScene.fingerConnector)
  }

  //touch release
  f.touches.$removed-> (root) {
    t = getRef (root.f.touches.$removed)
    Process d = CCall (findTouch, root, t)  
    delete d
  }

  //pinching requires to have exactly 2 touches, associate actions to change of state
  //native actions are required because the components must not be activated 
  //when building the tree
  Switch pinchSw (idle) {
    Component idle {
      NativeAction _ (delete_action, root, 0)
    }
    Component pinching {
	  NativeAction _ (pinch_action, root, 0)
    }
  }
  nbTouches == 2 ? "pinching" : "idle" => pinchSw.state

}

run root
run syshook
