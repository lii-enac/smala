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
#include "term_node.h"
namespace Smala
{

  class CtrlNode : public Node
  {
  public:
    CtrlNode (const location& loc);
    CtrlNode (const location& loc, const std::string &type, const std::string &name, const std::vector< std::pair<ParamType, std::string> > &arguments);
    CtrlNode (const location& loc, const std::string &type, const std::string &name, std::string in_act = "true", std::string out_act = "true");
    virtual ~CtrlNode ();

    Node* in () const;
    void set_in (Node *in);
    Node* out () const;
    void set_out (Node *out);

    void add_expression (std::vector<TermNode*> expression) { m_expression.insert (m_expression.begin (), expression.begin (), expression.end ()); }
    std::vector<TermNode*>& expression () { return m_expression; }
    void add_output_node (PathNode *n) { m_vout.push_back (n); }
    std::vector<PathNode*>& get_output_nodes () { return m_vout; }
    std::string& get_in_act () { return m_in_act; }
    std::string& get_out_act () { return m_out_act; }

  private:
    Node *m_in;
    Node *m_out;
    std::string m_in_act, m_out_act;
    std::vector<TermNode*> m_expression;
    std::vector<PathNode*> m_vout;
  };

} /* namespace Smala */
