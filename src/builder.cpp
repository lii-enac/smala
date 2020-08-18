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
#include "term_node.h"
#include "range_node.h"

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

  void
  Builder::build_preamble (std::ofstream &os)
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
            //std::string name = cur_node->name ();
            build_import (os, cur_node);
            break;
          }
        default: {}
        }
    }
    /* inline function */
    os << "inline\n";
    os << "double smala_deref(djnn::AbstractProperty& p)\n";
    os << "{ return p.get_double_value(); }\n\n";

    os << "inline\n";
    os << "std::string smala_deref(std::string p)\n";
    os << "{ return p; }\n\n";

    os << "inline\n";
    os << "double smala_deref(double p)\n";
    os << "{ return p; }\n\n";


    /* then the auto-generated native */
    for (auto e: m_ast.native_expression_list ()) {
      build_native_expression (os, e);
    }

    /* finally the user defined native */
    for (int i = 0; i < size; ++i) {
      cur_node = m_ast.preamble ().nodes ().at (i);
      switch (cur_node->node_type ()) {
        case NATIVE_ACTION:
          {
            build_native_action (os, cur_node);
            break;
          }
        case NATIVE_COLLECTION_ACTION:
          {
            build_native_collection_action (os, cur_node);
            break;
          }
        case NATIVE_CODE:
          {
            NativeCodeNode *n = static_cast<NativeCodeNode*> (cur_node);
            os << n->code () << std::endl;
            break;
          }
        default:
          build_node (os, cur_node);
        }
    }
  }

  void
  Builder::build_node (std::ofstream &os, Node *node)
  {
    m_curloc = node->location ();
    switch (node->node_type ())
      {
      case START_MAIN:
      {
        build_main_node (os);
        break;
      }
      case START_DEFINE:
      {
        build_define_node (os, node);
        break;
      }
      case START_ELSE:
      {
        push_ctxt ();
        indent  (os);
        os << "else {\n";
        m_indent++;
        break;
      }
      case START_ELSEIF:
      {
        indent  (os);
        os << "else ";
        break;
      }
      case START_IF:
      {
        push_ctxt ();
        indent  (os);
        os << "if (";
        m_in_static_expr = true;
        for (Node *n : node->get_expr_data ()) {
          build_node (os, n);
        }
        m_in_static_expr = false;
        break;
      }
      case BREAK:
      {
        indent (os);
        os << node->djnn_type() << ";\n";
        break;
      }
      case INCREMENT:
      {
        build_step (os, node, true);
        break;
      }
      case DECREMENT:
      {
        build_step (os, node, false);
        break;
      }
      case END_IF_EXPRESSION:
      {
        os << ") {\n";
        m_indent++;
        break;
      }
      case END_BLOCK:
      {
        m_indent--;
        indent (os);
        os << "}\n";
        pop_ctxt ();
        break;
      }
      case END_LOOP:
      {
        m_in_for = false;
        break;
      }
      case END_DEFINE:
      {
        build_end_define (os, node);
        break;
      }
      case END_CONTAINER:
      {
        BuildNode* n = m_parent_list.at (m_parent_list.size() - 1);
        m_parent_list.pop_back ();
        if (n) delete n;
        break;
      }
      case END_MAIN:
      {
        build_end_main (os, node);
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
        m_in_native_action = true;
        build_smala_native (os, node);
        break;
      }
      case END_NATIVE:
      {
        os << "}\n\n";
        BuildNode* n = m_parent_list.at (m_parent_list.size() - 1);
        m_parent_list.pop_back ();
        if (n) delete n;
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
      case END_SET_PROPERTY:
      {
        end_set_property (os, node);
        break;
      }
      case END_PROPERTY:
      {
        end_property (os, node);
        break;
      }
      case GET_PROPERTY:
      {
        get_property (os, node);
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
        os << ";\n";
        indent (os);
        os << m_parent_list.back ()->name () << "->add_child (" << m_cur_building_name << ", \""
           <<  m_parent_list.back ()->get_key (m_cur_building_name) << "\");\n";
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
        m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
        break;
      }
      case SET_PARENT:
      {
        //BuildNode* n = m_parent_list.at (m_parent_list.size() - 1);
        //m_parent_list.pop_back ();
        //if (n) delete n;
        SetParentNode* spn = static_cast<SetParentNode*> (node);
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
      case CONTAINER:
      {
        std::string new_name = build_simple_node (os, node);
        m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
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
      case TERM_NODE:
      {
        build_term_node (os, node);
        break;
      }
      case NATIVE_CODE:
      {
        NativeCodeNode *n = static_cast<NativeCodeNode*> (node);
        os << n->code () << std::endl;
        break;
      }
      case NEW_LINE:
      {
        build_new_line (os, static_cast<NewLineNode*> (node));
        break;
      }
      case NEW_VAR:
      {
        NewVarNode *n = static_cast<NewVarNode*> (node);
        if (!m_in_for)
          indent (os);
        print_type (os, n->type ());
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
            default:
              new_name =  "";
          }
        }
        if (m_parent_list.back ()->add_entry (n->var_name (), new_name) == 1 && node->duplicate_warning ())
          print_error_message (error_level::warning, "duplicated name: " + n->var_name (), 0);
        os << " " << new_name << " = ";
        break;
      }
      case DASH_ARRAY:
      {
        build_dash_array (os, static_cast<DashArrayNode*> (node));
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
        m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
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
  Builder::build_simple_node (std::ofstream &os, Node *node)
  {
    std::string constructor = get_constructor (node->djnn_type ());
    if (constructor == "Switch")
      m_in_switch = true;
    std::string name = node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";

    if (node->name ().compare ("_") == 0)
      node->set_name ("");
    std::string new_name;

    if (node->keep_name () && !node->name ().empty ())
      new_name = node->name ();
    else {
      new_name = "cpnt_" + std::to_string (m_cpnt_num++);
      if (constructor.compare ("IvyAccess") == 0) {
        new_name = "ivy_" + std::to_string (m_cpnt_num++);
      }
    }
    node->set_build_name (new_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1 && node->duplicate_warning ())
        print_error_message (error_level::warning, "duplicated name: " + node->name (), 0);
    }
      indent (os);
      std::string p_name = (node->parent () == nullptr || node->ignore_parent ()) ? m_null_symbol : node->parent ()->build_name ();
      print_start_component (os, new_name, constructor);
      os << " (" << p_name << ", " << name;
      if (node->has_arguments ())
        os << ", ";
      else
        os << ");\n";
    return new_name;
  }

  void
  Builder::build_range_node (std::ofstream &os, Node *node, const string& new_name)
  {
    RangeNode* n = static_cast<RangeNode*> (node);
    std::string name = node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";
    indent (os);
    std::string p_name = (node->parent () == nullptr || node->ignore_parent ()) ? m_null_symbol : node->parent ()->build_name ();
    print_start_component (os, new_name, "SwitchRangeBranch");
    os << " (" << p_name << ", " << name << ", ";
    for (auto term: n->lower_arg()) {
      build_term_node (os, term);
    }
    os << ", " << n->left_open () << ", ";
    for (auto term: n->upper_arg ()) {
      build_term_node (os, term);
    }
    os << ", " << n->right_open () << ");\n";
  }

  void
  Builder::indent (std::ofstream &os)
  {
    for (int i = 0; i < m_indent; i++)
      os << "\t";
  }

  void
  Builder::print_start_component (std::ofstream &os, const std::string &name, const std::string &constructor)
  {
    print_component_decl (os, name);
    os << " = ";
    print_component_constructor (os, constructor);
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
