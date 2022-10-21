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

#include "node.h"
#include "path_node.h"

namespace Smala {


  typedef enum ExprType
  {
    LITERAL,
    PATH_EXPR,
    STEP,
    FUNCTION,
    UNARY_OP,
    BINARY_OP,
    TERNARY_OP,
    ARRAY
  } expr_node_t;

class PathNode;
class ExprNode : public Node
{
public:
    ExprNode (const location &loc, const std::string &val, expr_node_t expr_type, SmalaType type = UNDEFINED) :
      Node (loc, EXPR_NODE), m_expr_node_type (expr_type), m_val (val), m_expr_type (type), m_enclosed_with_parenthesis (false) { }
    ~ExprNode () {}
    expr_node_t get_expr_node_type () { return m_expr_node_type; }
    smala_t get_expr_type () { return m_expr_type; }
    void set_expr_type (smala_t type) { m_expr_type = type; }
    void set_enclosed_with_parenthesis (bool v) { m_enclosed_with_parenthesis = v; }
    bool is_enclosed_with_parenthesis () { return m_enclosed_with_parenthesis; }
    virtual std::string get_val () { return m_val; }
    virtual void update_type () {}
    virtual void print () { std::cout << m_val; }
private:
    expr_node_t m_expr_node_type;
    std::string m_val;
    smala_t m_expr_type;
    bool m_enclosed_with_parenthesis;
};

class PathExprNode : public ExprNode
{
public:
    PathExprNode (const location &loc, PathNode* path, smala_t type = UNDEFINED) : ExprNode (loc, "", PATH_EXPR, type) {
      set_path (path);
    }
    ~PathExprNode () {}
    void print () override { std::cout << get_path()->build_string_repr (); }
};

class StepExprNode : public ExprNode
{
public:
    StepExprNode (const location &loc, PathNode* path, bool incr, smala_t type = UNDEFINED) : ExprNode (loc, "", STEP, type), m_incr (incr) { set_path (path); }
    ~StepExprNode () {}
    bool is_incr () { return m_incr; }
    void print () override { std::cout << get_path()->build_string_repr (); }
private:
    bool m_incr;
};

class FunctionExprNode : public ExprNode
{
public:
    FunctionExprNode (const location &loc, const std::string &val) : ExprNode (loc, val, FUNCTION) {}
    ~FunctionExprNode () {}
};

class UnaryExprNode : public ExprNode
{
public:
    UnaryExprNode (const location &loc, const std::string &val, ExprNode* child = nullptr) : ExprNode (loc, val, UNARY_OP), m_child_expr (child) { update_type (); }
    ~UnaryExprNode () {}
    ExprNode* get_child () { return m_child_expr; }
    void set_child (ExprNode* c) { m_child_expr = c; update_type ();}
    void update_type () override;
private:
    ExprNode* m_child_expr;
};

class BinaryExprNode : public ExprNode
{
public:
    BinaryExprNode (const location &loc, const std::string &val) :
      ExprNode (loc, val, BINARY_OP), m_left_child (nullptr), m_right_child (nullptr) {}
    BinaryExprNode (const location &loc, const std::string &val, ExprNode* left_child, ExprNode* right_child) :
      ExprNode (loc, val, BINARY_OP), m_left_child (left_child), m_right_child (right_child) {}
    ~BinaryExprNode () {}
    ExprNode* get_left_child () { return m_left_child; }
    ExprNode* get_right_child () { return m_right_child; }
    void set_left_child (ExprNode* c) { m_left_child = c; update_type (); }
    void set_right_child (ExprNode* c) { m_right_child = c; update_type (); }
    void update_type () override;
private:
    ExprNode *m_left_child, *m_right_child;
};

class TernaryExprNode : public ExprNode
{
public:
    TernaryExprNode (const location &loc) :
      ExprNode (loc, "", TERNARY_OP), m_condition (nullptr), m_left_child (nullptr), m_right_child (nullptr) {}
    TernaryExprNode (const location &loc, ExprNode* condition, ExprNode* left_child, ExprNode* right_child) :
      ExprNode (loc, "", TERNARY_OP), m_condition (condition), m_left_child (left_child), m_right_child (right_child) {}
    ~TernaryExprNode () {}
    ExprNode* get_condition () { return m_condition; }
    ExprNode* get_left_child () { return m_left_child; }
    ExprNode* get_right_child () { return m_right_child; }
    void set_condition (ExprNode* c) { m_condition = c; }
    void set_left_child (ExprNode* c) { m_left_child = c; update_type (); }
    void set_right_child (ExprNode* c) { m_right_child = c; update_type (); }
    void update_type () override;

private:
    ExprNode *m_condition, *m_left_child, *m_right_child;
};

class ArrayItemsNode : public ExprNode {
public:
  ArrayItemsNode (const location &loc, ArrayItemsNode* parent = nullptr) :
    ExprNode (loc, "", ARRAY), m_parent (parent) {}
  ~ArrayItemsNode () {}
  ArrayItemsNode* get_parent () { return m_parent; }
  void set_parent (ArrayItemsNode* parent) { m_parent = parent; }
  void add_item (ExprNode* n) { m_array.push_back (n); }
  std::vector<ExprNode*> get_items () { return m_array; }
  void print () {
    std::string sep = "";
    for (auto n : m_array) {
      std::cout << sep;
      n->print ();
      sep = ", ";
    }
    std::cout << "\n";
  }
private:
  ArrayItemsNode* m_parent;
  std::vector<ExprNode*> m_array;
};

class ArrayItemsHolder
{
  public:
    ArrayItemsHolder () {}
    ~ArrayItemsHolder () {}
    void add_item (ExprNode *n) { m_items.push_back (n); }
    void clear () { m_items.clear (); }
    std::vector<ExprNode*> get_items () { return m_items; }

  private:
    std::vector<ExprNode*> m_items;
};

class ArrayVarNode : public Node
{
public:
  ArrayVarNode (const location &loc, const std::string& name, ArrayItemsNode* items, SmalaType type, int dimension) :
    Node(loc, ARRAY_VAR), m_array (items), m_type (type), m_dimension (dimension), m_name (name) {}
  ~ArrayVarNode () {}
  ArrayItemsNode* get_array () { return m_array; }
  SmalaType get_array_type () { return m_type; }
  int get_dimension () { return m_dimension; }
  const std::string& get_name () { return m_name; }
private:
  ArrayItemsNode* m_array;
  SmalaType m_type;
  int m_dimension;
  std::string m_name;
};
}
