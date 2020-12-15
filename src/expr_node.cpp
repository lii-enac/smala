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

#include "expr_node.h"

namespace Smala {

    void
    UnaryExprNode::update_type ()
    {
      if (m_child_expr == nullptr) {
        set_expr_type (UNDEFINED);
        return;
      }
      if (get_val () == "!")
        set_expr_type(BOOL);
      else
       set_expr_type (m_child_expr->get_expr_type ());
    }

    static bool
    is_arithmetic_op (const std::string& op) {
      return op == "+" || op == "-" || op == "/" || op == "*";
    }

    void
    BinaryExprNode::update_type ()
    {
      if (!m_left_child || !m_right_child) {
        set_expr_type (UNDEFINED);
        return;
      }
      SmalaType l_t = m_left_child->get_expr_type ();
      SmalaType r_t = m_right_child->get_expr_type ();
      if (get_val () == "+" && (l_t == STRING || r_t == STRING)) {
        set_expr_type (STRING);
        return;
      }
      if (is_arithmetic_op (get_val ())) {
        set_expr_type (DOUBLE);
      } else {
        set_expr_type (BOOL);
      }
    }

    void
    TernaryExprNode::update_type ()
    {
      if (!m_left_child || !m_right_child) {
        set_expr_type (UNDEFINED);
        return;
      }
      SmalaType l_t = m_left_child->get_expr_type ();
      SmalaType r_t = m_right_child->get_expr_type ();
      if (l_t == STRING || r_t == STRING) {
        set_expr_type (STRING);
        return;
      }
      if (l_t <= DOUBLE && r_t <= DOUBLE) {
        set_expr_type (l_t < r_t? r_t : l_t);
        return;
      }
      set_expr_type (DOUBLE);
    }

}
