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
#include "operator_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "local_node.h"
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
            std::string name = cur_node->name ();
            build_import (os, name);
            break;
          }
        default: {}
        }
    }

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
        indent  (os);
        os << "if (";
        m_in_static_expr = true;
        for (Node *n : node->get_expression ()) {
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
      case CLONE:
        {
          clone (os, node);
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
      case CONTAINER:
        {
          std::string new_name = build_simple_node (os, node);
          m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
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
      case NEW_VAR:
      {
        NewVarNode *n = static_cast<NewVarNode*> (node);
        if (!m_in_for)
          indent (os);
        print_type (os, n->type ());
        std::string new_name;
        switch (n->type ()) {
          case INT:
          case DOUBLE:
          case STRING:
            new_name = "var_" + std::to_string (m_var_num++);
            break;
          case PROCESS:
            new_name = "var_" + std::to_string (m_cpnt_num++);
            break;
          default:
            new_name =  "";
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
      default:
        return;
      }
  }

  /* WARNING: this method is overridden in cpp_builder.cpp */
  void
  Builder::build_term_node (std::ofstream &os, Node *node)
  {
    TermNode *n = static_cast<TermNode*> (node);
    switch (n->arg_type ())
      {
      case FUNCTION_CALL:
      case SYMBOL:
      case VALUE:
        os << n->arg_value ();
        break;
      case VAR:
        {
          std::pair<std::string, std::vector<std::string>> p = parse_symbol (n->arg_value ());
          if (m_in_static_expr && m_in_set_text) {
            os << "((TextProperty*)";
          }
          if (p.second.empty ())
            os << p.first;
          else
            print_find_component (os, node, p);
          if (m_in_static_expr) {
            if (m_in_set_text)
              os << ")->get_value ()";
            else
              os << "->get_double_value ()";
          }
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


  std::string
  Builder::build_simple_node (std::ofstream &os, Node *node)
  {
    std::string constructor = get_constructor (node->djnn_type ());
    if (constructor == "Switch")
      m_in_switch = true;
    std::string name = node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";

    if (node->name ().compare ("_") == 0)
      node->set_name ("");
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));

    node->set_build_name (new_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1 && node->duplicate_warning ())
        print_error_message (error_level::warning, "duplicated name: " + node->name (), 0);
    }
    if (dynamic_cast<RangeNode*> (node) != nullptr) {
      build_range_node (os, node, new_name);
    } else {
      indent (os);
      std::string p_name = (node->parent () == nullptr || node->ignore_parent ()) ? m_null_symbol : node->parent ()->build_name ();
      print_start_component (os, new_name, constructor);
      os << " (" << p_name << ", " << name;
      if (node->has_arguments ())
        os << ", ";
      else
        os << ");\n";
    }
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
  Builder::build_control_node (std::ofstream &os, Node *node)
  {
    CtrlNode *ctrl = static_cast<CtrlNode*> (node);
    if (node->djnn_type ().compare ("FSMTransition") == 0) {
      build_transition_node (os, ctrl);
      return;
    }
    std::string constructor = get_constructor (node->djnn_type ());
    std::pair<std::string, std::vector <std::string>> src, dst;
    switch (ctrl->in ()->node_type ())
      {
      case BINARY_OP:
        {
          src.first = ctrl->in ()->build_name ();
          src.second.push_back ("\"result\"");
          break;
        }
      case UNARY_OP:
        {
          src.first = ctrl->in ()->build_name ();
          src.second.push_back ("\"output\"");
          break;
        }
      case LOCAL_NODE:
        {
          LocalNode *n = static_cast<LocalNode*> (ctrl->in ());
          src.first = n->root ()->build_name ();
          std::string path = n->path ();
          std::replace (path.begin (), path.end (), '.', '/');
          src.second.push_back("\"" + path + "\"");
          break;
        }
      case LITERAL:
        {
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          indent (os);
          print_start_component (os, new_name, get_constructor ("Double"));
          os << "(" << node->parent ()->build_name () << ", " << m_null_string << ", " << ctrl->in ()->name ()
              << ");\n";
          src.first = new_name;
          src.second.push_back (m_null_string);
          break;
        }
      case TEXT:
        {
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          indent (os);
          print_start_component (os, new_name, get_constructor ("String"));
          os << "(" << node->parent ()->build_name () << ", " << m_null_string << ", " << ctrl->in ()->name ()
              << ");\n";
          src.first = new_name;
          src.second.push_back (m_null_string);
          break;
        }
      default:
        if (!ctrl->in ()->build_name ().empty ()) {
          src.first = ctrl->in ()->build_name ();
          src.second.push_back (m_null_string);
        } else if (!ctrl->in ()->name ().empty ()) {
          src = parse_symbol (ctrl->in ()->name ());
          std::string prefix = "var_";
          bool _is_var = src.first.substr (0, prefix.size ()) == prefix;
          if (_is_var) {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            indent (os);
            print_start_component (os, new_name, get_constructor ("Double"));
            os << "(" << node->parent ()->build_name () << ", " << m_null_string << ", " << src.first << ");\n";
            src.first = new_name;
            src.second.push_back (m_null_string);
          }
        } else {
          print_error_message (error_level::warning, "anonymous component in input of control node", 1);
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
          dst.second.push_back ("\"" + path + "\"");
          break;
        }
      default:
        if (!ctrl->out ()->build_name ().empty ()) {
          dst.first = ctrl->out ()->build_name ();
          dst.second.push_back (m_null_string);
        } else if (!ctrl->out ()->name ().empty ()) {
          dst = parse_symbol (ctrl->out ()->name ());
        } else {
          print_error_message (error_level::error, "anonymous component in output of control node", 1);
        }
      }

    indent (os);
    if (!node->name ().empty ()) {
      std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
      m_parent_list.back ()->add_entry (node->name (), new_name);
      node->set_build_name (new_name);
      print_component_decl (os, new_name);
      os << " = ";
      node->set_name ("\"" + node->name () + "\"");
    } else {
      node->set_name (m_null_string);
    }

    print_component_constructor (os, constructor);
    os << " (" << node->parent ()->build_name () << ", " << node->name ();
    os << ", " << build_root_and_path (src) << ", " << build_root_and_path (dst);

    if (node->djnn_type ().compare ("Assignment") == 0 || node->djnn_type ().compare ("PausedAssignment") == 0) {
      os << ", " << node->args ().at (0).second;
    }
    os << ");\n";
  }

  void
  Builder::indent (std::ofstream &os)
  {
    for (int i = 0; i < m_indent; i++)
      os << "\t";
  }

  void
  Builder::print_find_component (std::ofstream &os, Node *n, const std::pair< std::string, std::vector<std::string> > &sym)
  {
    os << build_find_component (n, sym);
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

  const std::pair< std::string, std::vector<std::string> >
  Builder::parse_symbol (const std::string &symbol)
  {
    std::string str;
    std::size_t pos = symbol.find ('.');
    std::vector <std::string> children;
    if (pos == std::string::npos) {
      // first try to find the key in the current symtable
      str = m_parent_list.back ()->get_symbol (symbol);
      if (str.empty ()) {
        // then check if it is a Djnn symbol that is a key prefixed by DJN
        if (symbol.substr(0,3) == "DJN")
          return std::make_pair (symbol, children);
        else {
          // finally check if it is a Smala symbol
          str = m_type_manager->get_smala_symbol (symbol);
          if (!str.empty ())
            return std::make_pair (str, children);
        }
        // if everything fails, print an error message
        print_error_message (error_level::error, "Symbol not found: " + symbol, 1);
        return std::make_pair (symbol, children);
      } else
        return std::make_pair (str, children);
    } else {
      std::string root = m_parent_list.back ()->get_symbol (symbol.substr (0, pos));
      if (root.empty ()) {
        print_error_message (error_level::error, "Symbol not found: " + symbol.substr (0, pos), 1);
        return std::make_pair (symbol.substr (0, pos), children);
      }
      string path = symbol.substr (pos + 1);
      while (pos != std::string::npos) {
        pos = path.find_first_of ('.');
        if (pos == std::string::npos) {
          children.push_back (path);
        } else {
          string newKey = path.substr (0, pos);
          children.push_back (newKey);
          path = path.substr (pos + 1);
        }

      }
      //std::string path = "\"" + symbol.substr (pos + 1, symbol.length () - 1) + "\"";
      //std::replace (path.begin (), path.end (), '.', '/');
      return std::make_pair (root, children);
    }
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
