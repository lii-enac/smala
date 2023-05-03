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

#include <locale>
#include <algorithm>

#include "builder.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "native_action_node.h"
#include "newvar_node.h"
#include "set_parent_node.h"
#include "native_code_node.h"
#include "expr_node.h"
#include "range_node.h"

#define emit_compiler_info(OS) { indent(OS); OS << "// code emited by " << __PRETTY_FUNCTION__ << ":" << __FILE__ << ":" << __LINE__ << "\n"; }

namespace Smala
{
  BuildNode::BuildNode (const std::string &name, BuildNode *prev_node) :
      m_name (name), m_prev_node (prev_node)
  {
    m_sym_table = new std::map<std::string, std::string>;
  }

  BuildNode::BuildNode (const std::string &name) :
      m_name (name), m_prev_node (nullptr)
  {
    m_sym_table = new std::map<std::string, std::string> ();
  }

  BuildNode::BuildNode () :
      m_name (""), m_prev_node (nullptr)
  {
    m_sym_table = new std::map<std::string, std::string> ();
  }

  BuildNode::~BuildNode ()
  {
  }

  std::string
  BuildNode::get_symbol (const std::string &key) const
  {
    std::map<std::string, std::string>::const_iterator it;
    it = m_sym_table->find (key);
    if (it == m_sym_table->end ()) {
      if (m_prev_node)
        return m_prev_node->get_symbol (key);
      return "";
    } else
      return it->second;
  }
  std::string
  BuildNode::get_key (const std::string &value) const
  {
    //std::map<std::string, std::string>::const_iterator it;
    for (std::map<std::string, std::string>::iterator it=m_sym_table->begin(); it!=m_sym_table->end(); ++it) {
     if (it->second.compare (value) == 0)
       return it->first;
    }
    return "";
    }

  std::map<std::string, std::string>*
  BuildNode::sym_table ()
  {
    return m_sym_table;
  }

  int
  BuildNode::add_entry (const std::string &key, const std::string &value)
  {
    /* check for duplicated entry */
    std::map<std::string, std::string>::const_iterator it;
    it = m_sym_table->find (key);
    if (it == m_sym_table->end ()) {
      m_sym_table->insert (std::pair<std::string, std::string> (key, value));
      return 0;
    } else
      return 1;
  }

  const std::string&
  BuildNode::name () const
  {
    return m_name;
  }

  bool
  Builder::is_string (ExprNode *e)
  {
    switch (e->get_expr_node_type ())
      {
      case LITERAL:
        return e->get_expr_type () == STRING;
      case PATH_EXPR:
        return e->get_expr_type () == CAST_STRING;
      case STEP:
      case FUNCTION:
      case UNARY_OP:
        return false;
      case BINARY_OP:
        {
          BinaryExprNode *bin = (BinaryExprNode*) e;
          if (bin->get_val ().compare ("+") == 0
              || bin->get_val ().compare ("==") == 0)
            {
              return is_string (bin->get_left_child ())
                  || is_string (bin->get_right_child ());
            }
          else return false;
        }
      case TERNARY_OP:
        {
          TernaryExprNode *ter = (TernaryExprNode*) e;
          return is_string (ter->get_left_child ())
              || is_string (ter->get_right_child ());
        }
      case ARRAY:
        return false;
      }
    return false;
  }

  bool
  Builder::has_complex_term (PathNode* n)
  {
    std::vector<SubPathNode*> n_list = n->get_subpath_list ();
    for (auto n: n_list) {
      if (n->get_path_type () == EXPR) {
        if (n->get_expr()->get_expr_node_type () != LITERAL)
          return true;
      }
    }
    return false;
  }

  void
  Builder::push_ctxt (const string& parent_name)
  { //DBG;
    std::string the_parent_name;
    if (!parent_name.empty()) {
      the_parent_name = parent_name;
    } else {
      the_parent_name = m_parent_list.back ()->name ();
    }
    m_parent_list.push_back (new BuildNode (the_parent_name, m_parent_list.back ()));
  }

  void
  Builder::pop_ctxt ()
  { //DBG;
    BuildNode* n = m_parent_list.at (m_parent_list.size() - 1);
    m_parent_list.pop_back ();
    if (n) delete n;
  }


  void
  Builder::extract_leaves (std::vector<ExprNode*> &leaves, ExprNode *n)
  {
    switch (n->get_expr_node_type()) {
      case LITERAL:
      case PATH_EXPR:
      case STEP:
      case ARRAY:
        leaves.push_back (n);
        break;
      case FUNCTION:
        for (auto arg: ((FunctionExprNode*)n)->get_args()) {
          extract_leaves(leaves, arg);
        }
        break;
      case UNARY_OP:
        extract_leaves(leaves, ((UnaryExprNode*)n)->get_child());
        break;
      case BINARY_OP:
        extract_leaves(leaves, ((BinaryExprNode*)n)->get_left_child());
        extract_leaves(leaves, ((BinaryExprNode*)n)->get_right_child());
        break;
      case TERNARY_OP:
        extract_leaves(leaves, ((TernaryExprNode*)n)->get_condition());
        extract_leaves(leaves, ((TernaryExprNode*)n)->get_left_child());
        extract_leaves(leaves, ((TernaryExprNode*)n)->get_right_child());
        break;
    }
  }

  void
  Builder::build_preamble (std::ostream &os, bool debug)
  {
    int size = m_ast.preamble ().nodes ().size ();
    Node *cur_node;
    /* put first use and import */
    for (int i = 0; i < size; ++i) {
      cur_node = m_ast.preamble ().nodes ().at (i);
      switch (cur_node->node_type ())
        {
        case USE:
          {
            set_location (os, cur_node, debug);
            std::string str = cur_node->name ();
            std::locale loc;
            str[0] = std::toupper (str[0], loc);
            m_type_manager->add_global_symbols (str, m_parent_list.back ()->sym_table ());
            m_type_manager->init_types (str, m_types);
            str[0] = std::tolower (str[0], loc);
            build_use (os, str);
            break;
          }
        case IMPORT:
          {
            set_location (os, cur_node, debug);
            //std::string name = cur_node->name ();
            build_import (os, cur_node);
            break;
          }
        default: {}
        }
    }
    /* inline function */
    //build_post_import (os); // only in case of deref


    /* user-defined native code here so native expression can use it */
    for (int i = 0; i < size; ++i) {
      cur_node = m_ast.preamble ().nodes ().at (i);
      switch (cur_node->node_type ()) {
        case NATIVE_CODE:
          {
            set_location (os, cur_node, debug);
            emit_compiler_info (os);
            //if (debug) os << "\n#line " << cur_node->get_location().begin.line << " \"" << filename().substr(0, filename().length()-4) << ".sma\"" << std::endl;
            NativeCodeNode *n = dynamic_cast<NativeCodeNode*> (cur_node);
            os << n->code () << std::endl;
            break;
          }
        default: break;
        }
    }

    /* auto-generated native expressions */
    for (auto e: m_ast.native_expression_list ()) {
      build_native_expression (os, e);
    }

    /* user-defined native actions */
    for (int i = 0; i < size; ++i) {
      cur_node = m_ast.preamble ().nodes ().at (i);
      switch (cur_node->node_type ()) {
        case NATIVE_ACTION:
          {
            emit_compiler_info (os);
            set_location (os, cur_node, debug);
            build_native_action (os, cur_node);
            break;
          }
        case NATIVE_COLLECTION_ACTION:
          {
            emit_compiler_info (os);
            set_location (os, cur_node, debug);
            build_native_collection_action (os, cur_node);
            break;
          }
        case NATIVE_CODE: break; // already handled above
        default:
          //emit_compiler_info (os);
          build_node (os, cur_node);
        }
    }

  }

  void
  Builder::build_for_node (std::ostream &os, Node *node)
  {
    switch (node->node_type ())
      {
      case INCREMENT:
      {
        os << build_step ((ExprNode*) node);
        break;
      }
      case DECREMENT:
      {
        os << build_step ((ExprNode*) node);
        break;
      }
      case SET_PROPERTY:
      {
        set_property (os, node);
        break;
      }
      case SELF_SET_PROPERTY:
      {
        self_set_property (os, node);
        break;
      }
      case NEW_VAR:
      {
        NewVarNode *n = dynamic_cast<NewVarNode*> (node);
        auto expr_str = build_expr (n->get_args().at (0));
        //build_properties (os); // will be done in build_for
        indent (os);

        print_type (os, std::make_pair (n->type (), 0), n->get_args().at (0));
        std::string new_name;
        if (n->keep_name ())
          new_name = n->var_name ();
        else {
          switch (n->type ()) {
            case INT:
              new_name = "i_var_" + std::to_string (m_var_num++);
              break;
            case DOUBLE:
              new_name = "d_var_" + std::to_string (m_var_num++);
              break;
            case STRING:
              new_name = "s_var_" + std::to_string (m_var_num++);
              break;
            case PROCESS:
              new_name = "var_" + std::to_string (m_cpnt_num++);
              break;
            case ARRAY_T:
              new_name = "a_var_" + std::to_string (m_cpnt_num++);
              break;
            default:
              new_name =  "";
          }
        }
        if (m_parent_list.back ()->add_entry (n->var_name (), new_name) == 1 && node->duplicate_warning ())
          print_error_message (error_level::warning, "duplicated name: " + n->var_name (), 0);
        os << " " << new_name << " = " << expr_str;
        break;
      }
      case EXPR_NODE:
      {
        auto expr_str = build_expr ((ExprNode*) node);
        build_properties (os);
        indent (os);
        os << expr_str;
        break;
      }
      default:
        return;
      }
  }

  void
  Builder::build_node (std::ostream &os, Node *node)
  {
    m_curloc = node->error_location ();
    switch (node->node_type ())
      {
      case START_MAIN:
      {
        push_ctxt (); //DBG;
        build_main_node (os);
        break;
      }
      case END_MAIN:
      {
        build_end_main (os, node);
        pop_ctxt (); //DBG;
        break;
      }
      case START_DEFINE:
      {
        push_ctxt ("0"); //DBG;
        build_define_node (os, node);
        break;
      }
      case END_DEFINE:
      {
        build_end_define (os, node);
        pop_ctxt (); //DBG;
        break;
      }
      case CONTAINER: // beginning of Component etc. everything that starts with a '{' 
      {
        std::string new_name = build_simple_node (os, node);
        push_ctxt (new_name); //DBG;
        break;
      }
      case END_CONTAINER:
      {
        pop_ctxt (); //DBG;
        break;
      }
      case START_IF:
      {
        push_ctxt (); //DBG;
        //if (!m_after_else) {
        //  indent (os);
        //}
        build_start_if (os, node);
        m_after_else = false;
        break;
      }
      case START_ELSE:
      {
        // FIXME pop push ??
        push_ctxt (); //DBG;
        indent (os);
        build_start_else (os);
        m_indent++;
        break;
      }
      case START_ELSEIF:
      {
        // FIXME pop push ??
        indent (os);
        m_after_else = true;
        build_start_else_if (os);
        break;
      }
      case END_BLOCK:
      {
        m_indent--;
        indent (os);
        build_end_block (os);
        pop_ctxt (); //DBG;
        break;
      }
      case END_FOR_EVERY:
      {
        m_indent--;
        indent (os);
        build_end_for_every (os);
        pop_ctxt (); //DBG;
        break;
      }
      
      case BREAK:
      {
        indent (os);
        build_break (os, node);
        break;
      }
      case INCREMENT:
      {
        indent (os);
        os << build_step ((ExprNode*) node);
        end_line (os);
        break;
      }
      case DECREMENT:
      {
        indent (os);
        os << build_step ((ExprNode*) node);
        end_line (os);
        break;
      }
      
      case END_LOOP:
      {
        m_in_for = false;
        break;
      }
      case FOR:
      {
        build_for (os, node);
        break;
      }
      case FOR_EVERY:
      {
        build_for_every (os, node);
        break;
      }
      case WHILE:
      {
        build_while (os, node);
        break;
      }
      case PRINT:
      {
        build_print (os, node);
        break;
      }
      case SMALA_NATIVE:
      {
        // FIXME push ?!
        push_ctxt ();
        m_in_native_action = true;
        build_smala_native (os, node);
        break;
      }
      case END_NATIVE:
      {
        // FIXME pop ?
        build_end_native (os);
        pop_ctxt ();
        m_in_native_action = false;
        break;
      }
      case NATIVE_EXPRESSION:
      {
        build_native_expression_node (os, node);
        break;
      }
      case INSTRUCTION:
      {
        build_instruction (os, node);
        break;
      }
      case SET_PROPERTY:
      {
        set_property (os, node);
        break;
      }
      case SELF_SET_PROPERTY:
      {
        self_set_property (os, node);
        break;
      }
      case ALIAS:
      {
        alias (os, node);
        break;
      }
      case MERGE:
      {
        merge (os, node);
        break;
      }
      case REMOVE:
      {
        remove (os, node);
        break;
      }
      case MOVE_FIRST:
      {
        move (os, node, "FIRST");
        break;
      }
      case MOVE_BEFORE:
      {
        move (os, node, "BEFORE");
        break;
      }
      case MOVE_AFTER:
      {
        move (os, node, "AFTER");
        break;
      }
      case MOVE_END:
      {
        move (os, node, "LAST");
        break;
      }
      case END_ASSIGNMENT:
      {
        os << ";\n";
        break;
      }
      case ADD_CHILD:
      {
        add_child (os, node);
        break;
      }
      case END_ADD_CHILD:
      {
        build_end_add_child (os, node);
        break;
      }
      case ADD_CHILDREN_TO:
      {
        add_children_to (os, node);
        break;
      }
      case FSM:
      {
        std::string new_name = build_simple_node (os, node);
        push_ctxt (new_name);
        break;
      }
      case SET_PARENT:
      {
        //BuildNode* n = m_parent_list.at (m_parent_list.size() - 1);
        //m_parent_list.pop_back ();
        //if (n) delete n;
        SetParentNode* spn = dynamic_cast<SetParentNode*> (node);
        std::string parent;
        parent = m_parent_list.back()->name ();
        std::string name = spn->to_attach()->name();
        if (name == "_")
          name = "";
        fetch_add_child (os, parent, spn->to_attach()->build_name(), name);
        break;
      }
      case CONTROL:
      {
        build_control_node (os, node);
        break;
      }
      case SIMPLE:
      {
        build_simple_node (os, node);
        break;
      }
      case THIS:
      {
        build_this_node (os, node);
        break;
      }
      case TRANSITION:
      {
        build_transition_node (os, node);
        break;
      }
      case LAMBDA:
      {
        build_native_action_component (os, node);
        break;
      }
      case NATIVE_ACTION_CPNT:
      {
        build_native_action_component (os, node);
        break;
      }
      case EXPR_NODE:
      {
        auto expr = build_expr ((ExprNode*) node);
        build_properties (os);
        indent (os);
        os << expr;
        end_line (os);
        break;
      }
      case NATIVE_CODE:
      {
        emit_compiler_info (os);
        NativeCodeNode *n = dynamic_cast<NativeCodeNode*> (node);
        os << n->code () << std::endl;
        break;
      }
      case NEW_LINE:
      {
        //build_new_line (os, dynamic_cast<NewLineNode*> (node));
        break;
      }
      case NEW_VAR:
      {
        NewVarNode *n = dynamic_cast<NewVarNode*> (node);
        auto expr_str = build_expr (n->get_args().at (0));
        // if (!m_in_for)
        //   indent (os);
        build_properties (os);

        if (!m_in_for)
          indent (os);
        print_type (os, std::make_pair (n->type (), 0), n->get_args().at(0));
        std::string new_name;
        if (n->keep_name ())
          new_name = n->var_name ();
        else {
          switch (n->type ()) {
            case INT:
              new_name = "i_var_" + std::to_string (m_var_num++);
              break;
            case DOUBLE:
              new_name = "d_var_" + std::to_string (m_var_num++);
              break;
            case STRING:
              new_name = "s_var_" + std::to_string (m_var_num++);
              break;
            case PROCESS:
              new_name = "var_" + std::to_string (m_cpnt_num++);
              break;
            case ARRAY_T:
              new_name = "a_var_" + std::to_string (m_cpnt_num++);
              break;
            default:
              new_name =  "";
          }
        }
        if (m_parent_list.back ()->add_entry (n->var_name (), new_name) == 1 && node->duplicate_warning ())
          print_error_message (error_level::warning, "duplicated name: " + n->var_name (), 0);
        os << " " << new_name << " = " << expr_str;
        end_line(os);
        break;
      }
      case ARRAY_VAR:
      {
        ArrayVarNode* array = dynamic_cast<ArrayVarNode*> (node);
        if (array)
          build_array_var (os, array);
        break;
      }
      case RANGE:
      {
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        node->set_build_name (new_name);
        if (node->name ().compare ("_") != 0) {
          if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1 && node->duplicate_warning ())
                print_error_message (error_level::warning, "duplicated name: " + node->name (), 0);
        }
        build_range_node (os, node, new_name);
        push_ctxt (new_name);
        break;
      }
      case CAUSAL_DEP:
      {
        build_causal_dep (os, node);
        break;
      }
      default:
        return;
      }
  }

  std::string
  Builder::build_simple_node (std::ostream &os, Node *node)
  {
    std::string constructor = get_constructor (node->djnn_type ());

    if (constructor == "Switch")
      m_in_switch = true;

    std::string name = node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";

    if (node->name ().compare ("_") == 0)
      node->set_name ("");

    std::string var_name;

    if (node->keep_name () && !node->name ().empty ())
      var_name = node->name ();
    else {
      var_name = "cpnt_" + std::to_string (m_cpnt_num++);
      if (constructor.compare ("IvyAccess") == 0) {
        var_name = "ivy_" + std::to_string (m_cpnt_num++);
      }
    }
    node->set_build_name (var_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), var_name) == 1 && node->duplicate_warning ())
        print_error_message (error_level::warning, "duplicated name: " + node->name (), 0);
    }

    std::string parent_name = (node->parent () == nullptr || node->ignore_parent ()) ? m_null_symbol : node->parent ()->build_name ();
    
    //print_start_component (os, var_name, constructor);
    //build_component_arguments (os, parent_name, name, node);
    build_component (os, var_name, constructor, parent_name, name, node);
    return var_name;
  }

  void
  Builder::indent (std::ostream &os)
  {
    for (int i = 0; i < m_indent; i++)
      os << "\t";
  }

  std::string
  Builder::get_constructor (const std::string &type)
  {
    std::map<std::string, std::string>::iterator it;
    it = m_types.find (type);
    if (it == m_types.end ()) {
      return type;
    }
    return it->second;
  }

  void
  Builder::print_error_message (error_level::level_t level, const std::string& message, int error)
  {
    const std::string error_level_str[] =
      { "log", "warning", "error" };
    if (m_curloc != nullptr)  {   //TODO investigate why it can be null in lambda expr.
      std::cerr << m_curloc->file () << ":" << m_curloc->line () << ":" << m_curloc->position () << ": "
        << error_level_str[(int) level] << ": " << message << std::endl;
    }
    m_error |= error;
  }
} /* namespace Smala */
