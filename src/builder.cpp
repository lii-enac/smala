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

#include "builder.h"
#include "operator_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "ccall_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "local_node.h"
#include "arg_node.h"
#include "native_action_node.h"
#include <locale>
#include <algorithm>
#include "native_code_node.h"

namespace Smala {
  BuildNode::BuildNode (const std::string &name,
                        std::map<std::string, std::string> *sym_table) :
      m_name (name) {
    m_sym_table = new std::map<std::string, std::string>;
    m_sym_table->insert (sym_table->begin (), sym_table->end ());
  }

  BuildNode::BuildNode (const std::string &name) :
      m_name (name) {
    m_sym_table = new std::map<std::string, std::string> ();
  }

  BuildNode::BuildNode () :
      m_name ("") {
    m_sym_table = new std::map<std::string, std::string> ();
  }

  BuildNode::~BuildNode () {
  }

  void
  Builder::build_preamble (std::ofstream &os) {
    int size = m_ast.preamble ().nodes ().size ();
    Node *cur_node;
    for (int i = 0; i < size; ++i) {
      cur_node = m_ast.preamble ().nodes ().at (i);
      switch (cur_node->node_type ())
        {
        case USE:
          {
            std::string str = cur_node->name ();
            std::locale loc;
            str[0] = std::toupper (str[0], loc);
            m_type_manager->add_global_symbols (
                str, m_parent_list.back ().sym_table ());
            m_type_manager->init_types (str, m_types);
            str[0] = std::tolower (str[0], loc);
            build_use (os, str);
            break;
          }
        case IMPORT:
          {
            std::string name = cur_node->name ();
            build_import (os, name);
            break;
          }
        case NATIVE_ACTION:
          {
            build_native_action (os, cur_node);
            break;
          }
        case NATIVE_CODE: {
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
  Builder::build_node (std::ofstream &os, Node *node) {
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
      case END_DEFINE:
        {
          build_end_define (os, node);
          break;
        }
      case END_CONTAINER:
        {
          m_parent_list.pop_back ();
          break;
        }
      case SMALA_NATIVE:
        {
          build_smala_native (os, node);
          break;
        }
      case END_NATIVE:
        {
          os << "}\n\n";
          m_parent_list.pop_back ();
          break;
        }
      case CCALL:
        {
          build_native_code (os, node);
          break;
        }
      case CAT:
        {
          build_cat (os, node);
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
      case GET_PROPERTY:
        {
          get_property (os, node);
          break;
        }
      case MACRO:
        {
          if (!node->name ().empty () && !node->djnn_type ().empty ()) {
             if (m_parent_list.back ().add_entry (node->djnn_type (), node->name ()) == 1
                 && node->duplicate_warning ())
               print_error_message (error_level::warning,
                                    "duplicated macro: " + node->djnn_type (), 0);
           }
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
      case CLONE:
        {
          clone (os, node);
          break;
        }
      case END_CLONE:
        {
          m_indent--;
          indent (os);
          os << "}\n";
          m_parent_list.pop_back ();
          break;
        }
      case LOAD_XML:
        {
          load_xml (os, node);
          break;
        }
      case ADD_CHILD:
        {
          add_child (os, node);
          break;
        }
      case ADD_CHILDREN_TO:
        {
          add_children_to (os, node);
          break;
        }
      case FIND:
        {
          find (os, node);
          break;
        }
      case FSM:
      case SWITCH:
        {
          std::string new_name = build_simple_node (os, node, true);
          m_parent_list.push_back (
              BuildNode (new_name, m_parent_list.back ().sym_table ()));
          break;
        }
      case SET_PARENT:
        {
          std::string fsm = m_parent_list.back ().name ();
          m_parent_list.pop_back ();
          std::string parent = m_parent_list.back ().name ();
          std::string name = node->name ();
          fetch_add_child (os, parent, fsm, name);
          break;
        }
      case CONTROL:
        {
          build_control_node (os, node);
          break;
        }
      case SIMPLE:
        {
          build_simple_node (os, node, false);
          break;
        }
      case THIS:
        {
          build_this_node (os, node);
          break;
        }
      case CONTAINER:
        {
          std::string new_name = build_simple_node (os, node, false);
          m_parent_list.push_back (
              BuildNode (new_name, m_parent_list.back ().sym_table ()));
          break;
        }
      case NATIVE_ACTION_CPNT:
        {
          build_native_action_component (os, node);
          break;
        }
      case BINARY_OP:
        {
          build_binary_node (os, node);
          break;
        }
      case UNARY_OP:
        {
          build_unary_node (os, node);
          break;
        }
      case ARG_NODE:
        {
          build_arg_node (os, node);
          break;
        }
      case NATIVE_CODE:
        {
          NativeCodeNode *n = static_cast<NativeCodeNode*> (node);
          os << n->code () << std::endl;
          break;
        }
      case ACTIVATOR:
        {
          ActivatorNode *n = static_cast<ActivatorNode*> (node);
          build_activator (os, n);
          break;
        }
      case NEW_LINE:
        {
          build_new_line (os, static_cast<NewLineNode*> (node));
          break;
        }
      default:
        return;
      }
  }

  void
  Builder::build_arg_node (std::ofstream &os, Node *node) {
    ArgNode *n = static_cast<ArgNode*> (node);
    switch (n->arg_type ())
      {
      case SYMBOL:
      case VALUE:
        os << n->arg_value ();
        break;
      case VAR:
        {
          std::pair<std::string, std::string> p = parse_symbol (
              n->arg_value ());
          if (p.second.compare (m_null_string) == 0)
            os << p.first;
          else
            print_find_component (os, p.first, p.second);
          break;
        }
      case SMALA_NULL:
        {
          os << m_null_symbol;
          break;
        }
      case END:
        os << ");\n";
        break;
      default:
        return;
      }
  }

  void
  Builder::build_cat (std::ofstream &os, Node *node) {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *left = op->left ();
    Node *right = op->right ();
    std::string constructor = get_constructor (node->djnn_type ());
    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    node->set_build_name (new_name);
    m_parent_list.back ().add_entry (new_name, new_name);

    indent (os);
    print_start_component (os, new_name, constructor);
    os << " (" << m_parent_list.back ().name () << ", " << m_null_string
        << ");\n";
    if (!node->name ().empty ()) {
      m_parent_list.back ().add_entry (
          node->name (), build_find_component (new_name, "\"output\""));
    }
    indent (os);
    if (left->node_type () == LITERAL) {
      build_set_string (os, new_name, "\"head\"", left->name ());
      //os << get_constructor ("setString") << " (" << new_name << ", \"head\", "
      // << left->name () << ");\n";
    } else if (left->node_type () == PATH) {
      print_component_constructor (os, get_constructor ("Connector"));
      os << " (" << m_parent_list.back ().name () << ", " << m_null_string
          << ", ";
      std::pair<std::string, std::string> pair = parse_symbol (left->name ());
      os << pair.first << ", " << pair.second << ", " << new_name
          << ", \"head\");\n";
    } else {
      print_component_constructor (os, get_constructor ("Connector"));
      os << " (" << m_parent_list.back ().name () << ", " << m_null_string
          << ", " << left->build_name () << ", \"output\"" << ", " << new_name
          << ", \"head\");\n";
    }

    indent (os);
    if (right->node_type () == LITERAL) {
      build_set_string (os, new_name, "\"tail\"", right->name ());
      //os << get_constructor ("setString") << " (" << new_name << ", \"tail\", "
      //  << right->name () << ");\n";
    } else if (right->node_type () == PATH) {
      print_component_constructor (os, get_constructor ("Connector"));
      os << " (" << m_parent_list.back ().name () << ", " << m_null_string
          << ", ";
      std::pair<std::string, std::string> pair = parse_symbol (right->name ());
      os << pair.first << ", " << pair.second << ", " << new_name
          << ", \"tail\");\n";
    } else {
      print_component_constructor (os, get_constructor ("Connector"));
      os << " (" << m_parent_list.back ().name () << ", " << m_null_string
          << ", " << right->build_name () << ", \"output\"" << ", " << new_name
          << ", \"tail\");\n";
    }
  }

  std::string
  Builder::build_simple_node (std::ofstream &os, Node *node,
                              bool ignore_parent) {
    std::string constructor = get_constructor (node->djnn_type ());
    std::string name =
        node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";

    if (node->name ().compare ("_") == 0)
      node->set_name ("");
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));

    node->set_build_name (new_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ().add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    indent (os);
    std::string p_name =
        (node->parent () == nullptr || ignore_parent) ?
            m_null_symbol : node->parent ()->build_name ();
    print_start_component (os, new_name, constructor);
    os << " (" << p_name << ", " << name;
    if (node->has_arguments ())
      os << ", ";
    else
      os << ");\n";
    return new_name;
  }

  void
  Builder::build_control_node (std::ofstream &os, Node *node) {
    CtrlNode *ctrl = static_cast<CtrlNode*> (node);
    if (node->djnn_type ().compare ("FSMTransition") == 0) {
      build_transition_node (os, ctrl);
      return;
    }
    std::string constructor = get_constructor (node->djnn_type ());
    //cout << "type = " << node->djnn_type() << " constructor = "  << constructor << endl;
    std::pair<std::string, std::string> src, dst;

    switch (ctrl->in ()->node_type ())
      {
      case BINARY_OP:
        {
          src.first = ctrl->in ()->build_name ();
          src.second = "\"result\"";
          break;
        }
      case UNARY_OP:
        {
          src.first = ctrl->in ()->build_name ();
          src.second = "\"output\"";
          break;
        }
      case LOCAL_NODE:
        {
          LocalNode *n = static_cast<LocalNode*> (ctrl->in ());
          src.first = n->root ()->build_name ();
          std::string path = n->path ();
          std::replace (path.begin (), path.end (), '.', '/');
          src.second = "\"" + path + "\"";
          break;
        }
      case LITERAL:
        {
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          indent (os);
          print_start_component (os, new_name, get_constructor ("Double"));
          os << "(" << node->parent ()->build_name () << ", " << m_null_string
              << ", " << ctrl->in ()->name () << ");\n";
          src.first = new_name;
          src.second = m_null_string;
          break;
        }
      case TEXT:
        {
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          indent (os);
          print_start_component (os, new_name, get_constructor ("String"));
          os << "(" << node->parent ()->build_name () << ", " << m_null_string
              << ", " << ctrl->in ()->name () << ");\n";
          src.first = new_name;
          src.second = m_null_string;
          break;
        }
      default:
        if (!ctrl->in ()->build_name ().empty ()) {
          src.first = ctrl->in ()->build_name ();
          src.second = m_null_string;
        } else if (!ctrl->in ()->name ().empty ()) {
          src = parse_symbol (ctrl->in ()->name ());
          std::string prefix = "var_";
          bool _is_var = src.first.substr (0, prefix.size()) == prefix;
	  if (_is_var) {
	    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
	    indent (os);
	    print_start_component (os, new_name, get_constructor ("Double"));
	    os << "(" << node->parent ()->build_name () << ", " << m_null_string << ", " << src.first
		<< ");\n";
	    src.first = new_name;
	    src.second = m_null_string;
	  }
        } else {
          print_error_message (error_level::warning,
                               "anonymous component in input of control node",
                               1);
          m_error |= 1;
        }
      }

    switch (ctrl->out ()->node_type ())
      {
      case LOCAL_NODE:
        {
          LocalNode *n = static_cast<LocalNode*> (ctrl->out ());
          dst.first = n->root ()->build_name ();
          std::string path = n->path ();
          std::replace (path.begin (), path.end (), '.', '/');
          dst.second = "\"" + path + "\"";
          break;
        }
      default:
        if (!ctrl->out ()->build_name ().empty ()) {
          dst.first = ctrl->out ()->build_name ();
          dst.second = m_null_string;
        } else if (!ctrl->out ()->name ().empty ()) {
          dst = parse_symbol (ctrl->out ()->name ());
      } else {
        print_error_message (error_level::error,
                             "anonymous component in output of control node",
                             1);
      }
    }

  indent (os);
  if (!node->name ().empty ()) {
    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ().add_entry (node->name (), new_name);
    node->set_build_name (new_name);
    print_component_decl (os, new_name);
    os << " = ";
    node->set_name ("\"" + node->name () + "\"");
  } else {
    node->set_name (m_null_string);
  }

  print_component_constructor (os, constructor);
  os << " (" << node->parent ()->build_name () << ", " << node->name ();
  os << ", " << src.first << ", " << src.second << ", " << dst.first << ", "
      << dst.second;

  if (node->djnn_type ().compare ("Assignment") == 0
      || node->djnn_type ().compare ("PausedAssignment") == 0) {
    os << ", " << node->args ().at (0).second;
  }
  os << ");\n";
}

void
Builder::build_native_code (std::ofstream &os, Node *node) {
  NativeCallNode *n = static_cast<NativeCallNode*> (node);
  std::string new_name ("var_" + std::to_string (m_var_num++));
  indent (os);
  print_type (os, n->return_type ());
  os << " " << new_name << " = " << n->func_name () << " (";
  if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
    print_error_message (error_level::warning,
                         "duplicated name: " + node->name (), 0);
  if (!node->has_arguments ())
    os << ");\n";
}

std::string
BuildNode::get_symbol (const std::string &key) const {
  std::map<std::string, std::string>::const_iterator it;
  it = m_sym_table->find (key);
  if (it == m_sym_table->end ()) {
    return "";
  } else
    return it->second;
}

std::map<std::string, std::string>*
BuildNode::sym_table () {
  return m_sym_table;
}

int
BuildNode::add_entry (const std::string &key, const std::string &value) {
  /* check for duplicated entry */
  if (get_symbol (key).empty ()) {
    m_sym_table->insert (std::pair<std::string, std::string> (key, value));
    return 0;
  } else
    return 1;
}

const std::string&
BuildNode::name () const {
  return m_name;
}

void
Builder::indent (std::ofstream &os) {
  for (int i = 0; i < m_indent; i++)
    os << "\t";
}

void
Builder::print_find_component (std::ofstream &os, const std::string& first,
                               const std::string& second) {
  os << build_find_component (first, second);
}

void
Builder::print_start_component (std::ofstream &os, const std::string &name,
                                const std::string &constructor) {
  print_component_decl (os, name);
  os << " = ";
  print_component_constructor (os, constructor);
}

std::string
Builder::get_constructor (const std::string &type) {
  std::map<std::string, std::string>::iterator it;
  it = m_types.find (type);
  if (it == m_types.end ()) {
    print_error_message (error_level::error, "unknown type " + type, 1);
    return "";
  }
  return it->second;
}

const std::pair<std::string, std::string>
Builder::parse_symbol (const std::string &symbol) {
  std::string str;
  std::size_t pos = symbol.find ('.');
  if (pos == std::string::npos) {
    str = m_parent_list.back ().get_symbol (symbol);
    if (str.empty ()) {
      print_error_message (error_level::error, "Symbol not found: " + symbol,
                           1);
      return std::make_pair (symbol, m_null_string);
    } else
      return std::make_pair (str, m_null_string);
  } else {
    std::string root = m_parent_list.back ().get_symbol (
        symbol.substr (0, pos));
    if (root.empty ()) {
      print_error_message (error_level::error,
                           "Symbol not found: " + symbol.substr (0, pos), 1);
      return std::make_pair (symbol.substr (0, pos), m_null_string);
    }
    std::string path = "\"" + symbol.substr (pos + 1, symbol.length () - 1)
        + "\"";
    std::replace (path.begin (), path.end (), '.', '/');
    return std::make_pair (root, path);
  }
}
void
Builder::print_error_message (error_level::level_t level,
                              const std::string& message, int error) {
  const std::string error_level_str[] =
    { "log", "warning", "error" };
  std::cerr << m_curloc->file () << ":" << m_curloc->line () << ":" << m_curloc->position () << ": "
      << error_level_str[(int) level] << ": " << message << std::endl;
  m_error |= error;
}
} /* namespace Smala */
