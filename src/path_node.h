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

namespace Smala
{
  enum subpath_type {
    START,
    ITEM,
    REF,
    EXPR,
    WILD_CARD,
    PATH_LIST
  };

 enum cast_type {
  NO_CAST,
  BY_PROCESS,
  BY_VALUE,
  BY_REF
 };

  class PathNode;
  class ExprNode;
  class SubPathNode : public Node
  {
  public:
    SubPathNode (const location& loc): Node (loc, SUBPATH), m_expr (nullptr), m_path (""), m_type (ITEM), m_cast (NO_CAST) {}
    SubPathNode (const location& loc, const std::string &path, subpath_type type, cast_type cast = NO_CAST): Node (loc, SUBPATH), m_expr (nullptr), m_path (path), m_type (type), m_cast (cast) {}
    SubPathNode (const location& loc, ExprNode* n): Node (loc, SUBPATH), m_path (""), m_expr (n), m_type (EXPR), m_cast (NO_CAST) {}
    SubPathNode (const location& loc, const std::vector<PathNode*> path_list): Node (loc, SUBPATH), m_expr (), m_path (""), m_path_list (path_list), m_type (PATH_LIST), m_cast (NO_CAST) {}
    virtual ~SubPathNode () {}

    const std::string& get_subpath () const { return m_path; }
    void set_subpath (const std::string &path) { m_path = path; }
    subpath_type get_path_type () { return m_type; }
    void set_path_type (subpath_type type) { m_type = type; }
    std::vector<PathNode*> get_path_list () { return m_path_list; }
    ExprNode* get_expr () { return m_expr; }
    cast_type get_cast () { return m_cast; }

  private:
    ExprNode* m_expr;
    std::vector<PathNode*> m_path_list;
    std::string m_path;
    subpath_type m_type;
    cast_type m_cast;
  };

  class PathNode : public Node
    {
    public:
      PathNode (const location& loc) : Node (loc, PATH), m_cast (NO_CAST) {}
      PathNode (const location& loc, const std::vector<SubPathNode*> node_list) : Node (loc, PATH), m_path (node_list) { if (!node_list.empty()) m_cast = node_list.at(0)->get_cast ();}
      virtual ~PathNode () {}

      void add_subpath (SubPathNode* n) { if (m_path.empty ()) m_cast = n->get_cast(); m_path.push_back (n); };
      void set_cast (cast_type cast) { m_cast = cast; }
      cast_type get_cast () { return m_cast; }
      std::vector<SubPathNode*>& get_subpath_list () { return m_path; }
      const std::vector<SubPathNode*>& get_subpath_list () const { return m_path; }
      bool has_wild_card () { return m_path.back ()->get_path_type () == WILD_CARD; }
      bool has_path_list () { return m_path.back ()->get_path_type () == PATH_LIST; }

      std::string build_string_repr () const {
        std::string res;
        auto sep = "";
        for (auto & spn: get_subpath_list()) {
          res += sep + spn->get_subpath ();
          sep = ".";
        }
        return res;
      }

    private:
      std::vector<SubPathNode*> m_path;
      cast_type m_cast;
    };

} /* namespace Smala */
