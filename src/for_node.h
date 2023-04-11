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
    ForNode (const location& loc) : Node (loc, FOR), m_first (nullptr), m_second (nullptr), m_third (nullptr) {};
    virtual ~ForNode () {}

    void set_statements (Node *first, ExprNode* second, Node *third) { m_first = first; m_second = second; m_third = third; }
    Node* first_st () const { return m_first; }
    ExprNode* second_st () const { return m_second; }
    Node* third_st () const { return m_third; }

  private:
    Node *m_first;
    ExprNode *m_second;
    Node *m_third;
  };

} /* namespace Smala */
