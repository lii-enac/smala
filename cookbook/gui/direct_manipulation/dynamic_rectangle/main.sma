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
     src = release
     we need to found is parent: the rectagnle to delete it.
  */
  rect = find (src, "..")
  delete rect
}

// _action_
// create_rectangle (Process src, Process data)
// {  
//   //src = release (background.release)
//   //data = root

//   addChildrenTo data {
//     Rectangle rect ( $data.background.move.x, $data.background.move.y, 100, 100, 0, 0)
//     rect.release -> data.na_delete_rectangle
//   }
// }

_main_
Component root {
  
  Frame frame ("dynamic rectangle", 0, 0, 600, 600)
  Exit ex (0, 1)
  frame.close -> ex
  
  FillColor bg_color (#FFFFFF)
  Rectangle background (0, 0, $frame.width, $frame.height, 0, 0)

  FillColor fg_color (#373737)
  OutlineColor fg_color_outline (#FF0000)
  Text explanation (10, 20, "Press and release on the window to create a rectangle. then press and release a rectangle to delete it !")

  //NativeAction na_create_rectangle (create_rectangle, root, 1)
  //background.release -> na_create_rectangle

  NativeAction na_delete_rectangle (delete_rectangle, root, 1)

  background.release -> (root) {
    addChildrenTo root {
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

/*
problèmes:
1 - des rectangle apparaissent a des endroits ou l'on n'a pas cliqué 
2 - le graph n'est pas correctement nettoyer après le delete !!! --- le binding entre le rect.release et root.na_delete_rectangle ?? 
3 - pas d'accès a "src" depuis une lambda
4 - la génération du code de la native "smala" génère l'activation d'un composant en dehors de sont scope !!
5 - si on supprime cpnt5->activate () (le truc généré en trop par la question 4) --- le programme n'affiche plus rien. pourtant ce code semble inutile
6 - obligé de mettre un background pour éviter les abonnements a frame.release qui sont toujours lever et qui recrérait un rectangle sous un rectangle
7 - release x, y n'existent pas - on doit utiliser move.x, move.y (qui sont les derniers)
 */




