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

#include "native_expression_node.h"

namespace Smala
{

  NativeExpressionNode::NativeExpressionNode () :
      Node (), _paused (false), _is_connector (false), _is_model (true)
  {
    set_node_type (NATIVE_EXPRESSION);
  }

  NativeExpressionNode::NativeExpressionNode (std::vector<ArgNode*> expression, bool paused, bool is_connector, bool is_model) :
      Node (), _expression (expression), _paused (paused), _is_connector (is_connector), _is_model (is_model)
  {
    set_node_type (NATIVE_EXPRESSION);
  }

  NativeExpressionNode::~NativeExpressionNode ()
  {}

  void
  NativeExpressionNode::add_output_node (const std::string &n)
  {
    _output_nodes.push_back (n);
  }

  std::vector<ArgNode*>
  NativeExpressionNode::get_expression ()
  {
    return _expression;
  }

  std::vector<std::string>
  NativeExpressionNode::get_output_nodes ()
  {
    return _output_nodes;
  }
} /* namespace Smala */
