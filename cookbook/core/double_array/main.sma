/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Stephane Conversy <stephane.conversy@enac.fr>
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

_native_code_
%{
    #include <iostream>
    #include "core/property/template_property_array.h"

    void print_vec (vector< vector < vector <string> > > v) {
        for (auto i: v) {
          for (auto j: i) {
            for (auto k: j) {
                std::cout << k << " ";
            }
            std::cout << "\n";
          }
          std::cout << "\n";
        }
        std::cout <<"\n";
    }
%}

_main_
Component root {
    Frame f ("f", 0, 0, 500, 600)
    Exit ex (0, 1)
    f.close -> ex

    Rectangle r2 (0, 100, 100, 100)
    int[][] test = [[5, 6, 15],[2, 6], [3, 7]]
    string[][][] test_s = [[["test", "test2", "test2.5"],["test3", "test4"]],[["test5","test6"], ["test7","test8"]]]
    Process[] test_p = [ex, r2.width]
    print_vec (test_s)
}
