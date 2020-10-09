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

#include <vector>
#include <map>
#include <istream>

#include "scanner.h"

#include "parser.hpp"
#include "preamble.h"
#include "native_action_node.h"
#include "native_collection_action_node.h"
#include "native_expression_node.h"
#include "native_code_node.h"

namespace Smala {

class Node;
class Preamble;

class Ast {
public:
    Ast ();
    ~Ast ();
    void clear();
    void add_node (Node *node);
    void remove_node (Node *node);
    void add_use (const location& loc, const std::string &val);
    void add_import (const location& loc, PathNode *path);
    void add_native_action (const location& loc, const std::string &action_name, const std::string &parm_name, const std::string &code);
    void add_native_collection_action (const location& loc, const std::string &action_name, const std::string &list_name, const std::string &parm_name, const std::string &code);
    void add_native_code (const location& loc, const std::string &code);
    void add_native_expression (NativeExpressionNode *node);
    void add_define_node (Node *defineNode);
    void set_root_node (Node* root) { m_root = root; }
    Node* get_root_node () { return m_root; }
    void set_is_main (bool val);
    void set_in_preamble (bool val) { m_in_preamble = val; }
    void end_preamble ();

    const std::vector<Node*> define_node_list () const;
    const std::vector<NativeExpressionNode*> native_expression_list () const { return m_native_expression_list; }
    const std::vector<Node*>& node_list () const;
    const Preamble& preamble () const;
    bool is_main () const;
    
private:
    Preamble m_preamble;
    std::vector<NativeActionNode*> m_native_list;
    std::vector<NativeCollectionActionNode*> m_native_collection_list;
    std::vector<NativeExpressionNode*> m_native_expression_list;
    std::vector<SmalaNative*> m_smala_native;
    std::vector<Node*> m_node_list;
    std::vector<Node*> m_define_nodes;
    bool m_is_main;
    bool m_in_preamble;
    Node* m_root;
};

}
