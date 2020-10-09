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

  class ForNode : public Node
  {
  public:
    ForNode (const location& loc) : Node (loc, FOR), m_first (nullptr), m_third (nullptr) {};
    virtual ~ForNode () {}

    Node* first_st () const { return m_first; }
    void set_first_st (Node *first) { m_first = first; }
    Node* third_st () const { return m_third; }
    void set_third_st (Node *third) { m_third = third; }
    void set_expression (std::vector<Node*>& nodes) { m_expression = nodes; }
    std::vector<Node*>& get_expression () { return m_expression; }

  private:
    Node* m_first;
    Node* m_third;
    std::vector<Node*> m_expression;
  };

} /* namespace Smala */
