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
      Node (NATIVE_EXPRESSION), _paused (false), _is_connector (false), _is_model (true)
  {
  }

  NativeExpressionNode::NativeExpressionNode (std::vector<TermNode*> expression, bool paused, bool is_connector, bool is_model) :
      Node (NATIVE_EXPRESSION), _expression (expression), _paused (paused), _is_connector (is_connector), _is_model (is_model)
  {
  }

  NativeExpressionNode::~NativeExpressionNode ()
  {}

  void
  NativeExpressionNode::add_output_node (PathNode *n)
  {
    _output_nodes.push_back (n);
  }

  std::vector<TermNode*>&
  NativeExpressionNode::get_expression ()
  {
    return _expression;
  }

  std::vector<PathNode*>&
  NativeExpressionNode::get_output_nodes ()
  {
    return _output_nodes;
  }
} /* namespace Smala */
