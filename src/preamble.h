/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

#pragma once

#include <string>
#include <vector>

#include "node.h"
#include "native_code_node.h"
#include "path_node.h"

namespace Smala {

class Preamble
{
public:
    Preamble ();
    ~Preamble ();

    std::vector<Node*> nodes () const;
    std::vector<std::string> import () const;
    std::vector<std::string> use () const;
    void add_node (Node *node);
    void remove_node (Node *node);
    void add_import (const location& loc, PathNode *path);
    void add_use (const location& loc, const std::string &use);

    void clear ();
    
private:
    std::vector<Node *> m_node_list;
    std::vector<std::string> m_import_list;
    std::vector<std::string> m_use_list;
};

}
