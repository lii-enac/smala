use core
use base


// Undo/Redo Principles

// An "Interaction" allows the user to create, delete, or directly manipulate objects.
// At the "end" of an "Interaction", the "Interaction" should create an "Action" and add it to the actions list in the UndoRedoManager

// Each "Action" should react to the "undo", "redo" and "del" spikes
// (Note: an Action should be considered part of the Model, as per
// Jean-Daniel Fekete. Les trois services du noyau sémantique indispensables à l’IHM.
// 8èmes journées sur l’ingénierie de l’Interaction Homme-Machine (IHM 96), Sep 1996, Grenoble, France. pp.45-50. hal-00911555.)
// The undo/redo manager follows a typical undo/redo policy, where a new action after a few undo's will clear the list of undone actions
// (Not really usable, we should make it better...)

// Creation and Deletion management
// Undoing a Creation should not erase the involved objects, as this will prevent redoing subsequent actions such as Move, CreateArrow etc.
// (Move or CreateArrow needs pointers to the objects, which would be different in case of re-creation)
// A User-triggered Deletion should be handled similarly: Deletion should not actually delete an object to allow undoing the Deletion
// We simulate undoing of Deletion or Creation by removing the graphical objects from the view, (and moving them into a hidden Component in the case of Deletion)
// undoing and redoing consist in moving the graphical objects back and forth from a hidden component to a visible one (the scene)
// We still need to actually delete objects whenever the user clears the list of actions, which should be done by each Action through the 'del' spike

// TODO: ensure all Create/Delete Actions actually implement 'del'
// Note: currently, the action list is unbounded, which might be a problem with lot of actions.
// In practice, ending a magnitude session will clear this list...


_define_
UndoRedoManager ()
{
    List actions // list of actions
    Int current(0) // cursor position into the list of actions, decremented (resp. incremented) with undo (resp. redo)
    Spike undo
    Spike redo

    root = find(this, "//")

    //TextPrinter tp

    // whenever a new action is added to the list, make the cursor point to this action
    actions.$added -> added_: {
        current + 1 =: current
    }
    //debug
    // added_ -> (this) {
    //     print ("LIST actions size :" + this.actions.size)
    // }

    //current =:> tp.input

    // after a few undo's, the user might perform a new action
    // in this case this spike should be triggered to erase all actions that have been undone
    // this is the traditional policy in interactive applications, though it could be better usability-wise
    Spike removeActionsStartingFromCurrent
    removeActionsStartingFromCurrent -> remove_part1 : (this) {
        //print ("removeActionsStartingFromCurrent - remove_part1 - allinone")
        for (int i = this.actions.size; i > this.current; i--) {
            notify this.actions.[i].del // notify deletion so the action has a chance to clean-up its content
            graph_exec () // force sync on notify
            delete this.actions.[i]
            graph_exec ()
        }
        //print ("removeActionsStartingFromCurrent AFTER - actions size :" + this.actions.size)
    }

    // management of the position of the cursor in the action list
    Bool bempty(1)
    Bool bbeg(0)
    Bool bmiddle(0)
    Bool bend(0)

    added_ -> update_: {
        actions.size == 0 =: bempty
        !bempty && (current == 0) =: bbeg
        !bempty && (current == actions.size) =: bend
        !bempty && (!bbeg && !bend) =: bmiddle
    }
    current -> update_

    // TextPrinter tp2
    // "--- empty:" + toString(bempty) + " current:" + toString(current) + " size:" + toString(actions.size) + " -- bbeg:" + toString(bbeg) + " bmid:" + toString(bmiddle) + " bend:" + toString(bend) =:> tp2.input
    
    //_DEBUG_SEE_ACTIVATION_SEQUENCE = 1

    // management of the possibility of undoing or redoing, depending on the position of the cursor in the list
    FSM myfsm {
        State empty
        State end {
            undo -> (this) {
                action = &this.actions.[this.current]
                notify action.undo
                this.current = this.current - 1
            }
        }
        State beg {
            redo -> (this) {
                this.current = this.current + 1
                action = &this.actions.[this.current]
                notify action.redo
            }
        } 
        State middle {
            undo -> (this) {
                action = &this.actions.[this.current]
                notify action.undo
                this.current = this.current - 1
            }
            redo -> (this) {
                this.current = this.current + 1
                action = &this.actions.[this.current]
                notify action.redo
            }
        }
        empty -> end (bend.true)
        
        end -> beg (bbeg.true)
        end -> middle (bmiddle.true)

        middle -> end (bend.true)
        middle -> beg (bbeg.true)

        beg -> end (bend.true)
        beg -> middle (bmiddle.true)
        
    }
    // TextPrinter tp1
    // myfsm.state =:> tp1.input
}
