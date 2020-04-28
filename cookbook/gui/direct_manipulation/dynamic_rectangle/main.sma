/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

_action_
delete_rectangle (Process src, Process data)
{
  /*
     src = release (rect.release)
     we need to found is parent/parent : the grect to delete at once the rectangle + the binding
  */
  grect = find (&src, "../..")
  delete grect
}

// _action_
// create_rectangle (Process src, Process data)
// {  
//   //src = release (background.release)
//   //data = root

//   addChildrenTo data {
//     Component grect {
//        Rectangle rect ( $root.background.move.x, $root.background.move.y, 100, 100, 0, 0)
//        rect.release -> root.na_delete_rectangle
//      }
//   }
// }

_main_
Component root {
  
  Frame frame ("dynamic rectangle", 0, 0, 600, 600)
  Exit ex (0, 1)
  frame.close -> ex
  
  FillColor bg_color (#FFFFFF)
  Rectangle background (0, 0, $frame.width, $frame.height, 0, 0)
  /* if you want to bind the size of the background with the size of your window */
  frame.width => background.width
  frame.height => background.height


  FillColor fg_color (#373737)
  OutlineColor fg_color_outline (#FF0000)
  Text explanation1 (10, 20, "Press and release on the window to create a rectangle")
  Text explanation2 (10, 40, "then press and release a rectangle to delete it !")

  //NativeAction na_create_rectangle (create_rectangle, root, 1)
  //background.release -> na_create_rectangle

  NativeAction na_delete_rectangle (delete_rectangle, root, 1)

  background.release -> (root) {
    addChildrenTo root {
      /* we have to create a group to delete at once the rect and the associated binding */
      Component grect {
        Rectangle rect ( $root.background.move.x, $root.background.move.y, 100, 100, 0, 0)
        rect.release -> root.na_delete_rectangle

        // should work, no !!
        // rect.release -> () {
        //   rect = find (src, "..")
        //   delete rect
        // }
      }
    }
  }
  
}

/*
problèmes:
1 - FIXME - des rectangle apparaissent a des endroits ou l'on n'a pas cliqué du au TOUCHPAD du mac, aucun soucis avec une souris !!
2 - TODO - pas d'accès a "src" depuis une lambda. Ajouter le composant "_src_" depuis une smala
3 - TODO - la génération du code de la native "smala" génère l'activation d'un composant en dehors de sont scope !! - A CORRIGER dans parser.y end_main root = nullptr
4 - FIXED - dependance a notre model - obligé de mettre un background pour éviter les abonnements a frame.release qui sont toujours lever et qui recrérait un rectangle sous un rectangle
5 - FIXED - faire un groupe - le graph n'est pas correctement nettoyer après le delete !!! --- le binding entre le rect.release et root.na_delete_rectangle ?? 
 */




