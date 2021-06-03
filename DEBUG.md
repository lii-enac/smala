# README: DEBUGGING WITH SMALA
_____________________________

You can use several option to debug on smala :

## picking view infos :
    - a new frame with interactive object beside (right) the main frame

  _DEBUG_SEE_COLOR_PICKING_VIEW = 0|1
 
## detect cycle in graph and display informations

  _DEBUG_GRAPH_CYCLE_DETECT = 0|1;

## Activation Sequence 
  - display order in which component are activated and deactivated by GRAPH_EXEC

  _DEBUG_SEE_ACTIVATION_SEQUENCE = 0|1
  _DEBUG_SEE_ACTIVATION_SEQUENCE_TARGET_TIME_US  = 1000 // microsecond
  _DEBUG_SEE_ACTIVATION_SEQUENCE_ONLY_TARGETED = 1|0 // will display only the targeted process or full_stack (O)

## example : 

_main_
Component root {

  _DEBUG_SEE_COLOR_PICKING_VIEW = 1
  _DEBUG_GRAPH_CYCLE_DETECT = 1;
  _DEBUG_SEE_ACTIVATION_SEQUENCE = 1
  _DEBUG_SEE_ACTIVATION_SEQUENCE_TARGET_TIME_US  = 1000
  _DEBUG_SEE_ACTIVATION_SEQUENCE_ONLY_TARGETED = 1
  
  Frame f ("App", 0, 0, 600, 600)
  ...
}


## others tools :
	- *dump* XXXX : will dump the an ASCII tree from XXXX in the Terminal.
	- *print* "XXXX" : will print "XXXX" at each construction time



