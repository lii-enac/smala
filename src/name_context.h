/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2020)
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
#include "path_node.h"
#include "term_node.h"

namespace Smala {

class NameContext
{
public:
    NameContext () : m_in_expr (false) { }
    std::vector<SubPathNode*>  path () { return m_path; };
    std::vector<TermNode*> terms () { return m_term; }
    void add_subpath (SubPathNode* n) { m_path.push_back (n); }
    void add_term (TermNode* t) { m_term.push_back (t); }
    void remove_term (TermNode* t) {
      std::vector<TermNode*>::iterator it = find (m_term.begin(), m_term.end(), t);
      if (it != m_term.end()) {
        m_term.erase (it);
      }
    }
    void set_in_expr (bool v) { m_in_expr = v; }
    bool in_expr () { return m_in_expr; }
    void build_and_add_expression () {
      add_subpath (new SubPathNode (m_term));
      m_term.clear ();
    }

    ~NameContext () {}

private:
    std::vector<SubPathNode*> m_path;
    std::vector<TermNode*> m_term;
    bool m_in_expr;
};

}
