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

#include "node.h"

namespace Smala
{
  enum subpath_type {
    START,
    ITEM,
    REF,
    EXPR
  };

 enum cast_type {
  NO_CAST,
  BY_PROCESS,
  BY_VALUE,
  BY_REF
 };

  class SubPathNode : public Node
  {
  public:
    SubPathNode (): Node (SUBPATH), m_path (""), m_type (ITEM), m_cast (NO_CAST) {}
    SubPathNode (const std::string &path, subpath_type type, cast_type cast = NO_CAST): Node (SUBPATH), m_path (path), m_type (type), m_cast (cast) {}
    SubPathNode (const std::vector<TermNode*> node_list): Node (SUBPATH), m_path (""), m_expr (node_list), m_type (EXPR), m_cast (NO_CAST) {}
    virtual ~SubPathNode () {}

    const std::string& get_subpath () const { return m_path; }
    void set_subpath (const std::string &path) { m_path = path; }
    subpath_type get_path_type () { return m_type; }
    std::vector<TermNode*> get_expr () { return m_expr; }
    cast_type get_cast () { return m_cast; }

  private:
    std::vector<TermNode*> m_expr;
    std::string m_path;
    subpath_type m_type;
    cast_type m_cast;
  };

  class PathNode : public Node
    {
    public:
      PathNode () : Node (PATH), m_cast (NO_CAST) {}
      PathNode (const std::vector<SubPathNode*> node_list) : Node (PATH), m_path (node_list) { if (!node_list.empty()) m_cast = node_list.at(0)->get_cast ();}
      virtual ~PathNode () {}

      void add_subpath (SubPathNode* n) { if (m_path.empty ()) m_cast = n->get_cast(); m_path.push_back (n); };
      void set_cast (cast_type cast) { m_cast = cast; }
      cast_type get_cast () { return m_cast; }
      std::vector<SubPathNode*> get_subpath_list () { return m_path; }

    private:
      std::vector<SubPathNode*> m_path;
      cast_type m_cast;
    };

} /* namespace Smala */