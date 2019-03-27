/*
*  Multitouch touch app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2018-2019)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Sébastien Leriche <sebastien.leriche@enac.fr>
*    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/

use core
use base
use gui

_native_code_
%{
  #include <map>

  //map to store the various fingerConnector components  
  std::map<Process*, Process*> fcList;

  //store in the map
  Process* addTouch (Process* added, Process* fingerConnector) {
    fcList.insert(std::pair<Process*, Process*> (added, fingerConnector));
  }

  //find in the map which fingerConnector has to be removed
  Process* findTouch (Process* removed) {
    std::map<Process*, Process*>::iterator it;
    
    it = fcList.find(removed);
    Process* found;
    if (it != fcList.end()) {
      found = it->second; 
      fcList.erase(it);
    }

    return found;
  }
%}

_main_
Component root
{
  Frame f ("my frame", 0, 0, 1000, 1000)

  NoFill _
  OutlineColor _ (100,100,255)
  OutlineWidth _ (10)
  OutlineOpacity _ (0.5)

  f.touches.$added-> (root) {
    t = getRef (root.f.touches.$added)
    addChildrenTo root {
      Component fingerConnector {
        Circle finger (-100, -100, 100)
        t.x => root.fingerConnector.finger.cx
        t.y => root.fingerConnector.finger.cy
      }
    }
    Process plop = CCall (addTouch, t, root.fingerConnector)
  }

  f.touches.$removed-> (root) {
    t = getRef (root.f.touches.$removed)
    Process d = CCall (findTouch, t)  
    delete d
  }
}

run root
run syshook
