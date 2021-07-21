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

#include "expr_node.h"

namespace Smala
{

  class NativeExpressionNode : public Node
  {
  public:
    NativeExpressionNode (const location& loc);
    NativeExpressionNode (const location& loc, ExprNode* expression, bool paused, bool lazy, bool is_connector, bool is_model = true);
    virtual ~NativeExpressionNode ();
    void add_output_node (PathNode* output_node);
    ExprNode* get_expression ();
    std::vector<PathNode*>& get_output_nodes ();
    void set_build_name (const std::string &n) { _build_name = n; }
    const std::string& get_build_name () const { return _build_name; }
    bool is_connector () { return _is_connector; }
    bool is_paused () { return _paused; }
    bool is_lazy () { return _lazy; }
    bool is_model () { return _is_model; }
  private:
    ExprNode* _expression;
    std::vector<PathNode*> _output_nodes;
    bool _paused, _lazy, _is_connector, _is_model;
    std::string _build_name;
  };

} /* namespace Smala */
