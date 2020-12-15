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
#include <map>

#include "node.h"
#include "path_node.h"

namespace Smala {

class BuildNameContext
{
public:
    BuildNameContext () : m_in_expr (false) { }
    std::vector<SubPathNode*>  path () { return m_path; };
    void add_subpath (SubPathNode* n) { m_path.push_back (n); }
    bool in_expr () { return m_in_expr; }
    void build_and_add_expression (const location& loc, ExprNode *expr) {
      add_subpath (new SubPathNode (loc, expr));
      m_in_expr = true;
    }

    ~BuildNameContext () {}

private:
    std::vector<SubPathNode*> m_path;
    bool m_in_expr;
};

class SymTable
{
public:
  SymTable ();
  SymTable (SymTable* t): _prev (t) {}
  ~SymTable () {}
  void add_global_sym (const std::string& key, smala_t type);
  int add_sym (const location& loc, const std::string& key, smala_t type);
  bool exists (const std::string &key) const;
  SmalaType get_type (const std::string &key) const;
private:
  std::map<std::string, SmalaType> _t;
  SymTable *_prev;
};
}
