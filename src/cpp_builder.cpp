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

#include "cpp_builder.h"
#include "native_expression_node.h"
#include "operator_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "local_node.h"
#include "cpp_type_manager.h"
#include "for_node.h"

#include <locale>
#include <algorithm>

namespace Smala
{

  CPPBuilder::CPPBuilder () :
      Builder (), m_display_initialized (false)
  {
    m_type_manager = new CPPTypeManager ();
    m_null_symbol = "nullptr";
    m_null_string = "\"\"";
  }

  CPPBuilder::~CPPBuilder ()
  {
  }

  int
  CPPBuilder::build (const Ast &ast, const std::string &builddir,
                     const std::string &prefix)
  {
    m_indent = 0;
    m_cpnt_num = 0;
    m_var_num = 0;
    m_ast = ast;
    m_types.clear ();
    m_parent_list.clear ();
    m_parent_list.push_back (new BuildNode ("nullptr")); // the first parent is null
    if (!ast.is_main ())
      build_define (prefix);
    m_filename = std::string (prefix) + ".cpp";
    std::ofstream os (prefix + ".cpp");
    os << "#include <iostream>\n";
    os << "#include <string>\n";
    os << "#include \"core/core-dev.h\"\n";
    os << "#include \"exec_env/exec_env.h\"\n";
    os << "using namespace std;\nusing namespace djnn;\n\n";

    int size = m_ast.preamble ().import ().size ();
    for (int i = 0; i < size; ++i) {
      /* add the import name to the possible types */
      std::string name = m_ast.preamble ().import ().at (i);
      m_types.insert (std::pair<std::string, std::string> (name, name));
      m_import_types.insert (std::pair<std::string, std::string> (name, name));
    }
    build_preamble (os);

    size = m_ast.node_list ().size ();
    for (int i = 0; i < size; ++i) {
      build_node (os, m_ast.node_list ().at (i));
    }
    if (m_ast.is_main ())
      os << "}\n";
    os.close ();
    return m_error;
  }

  void
  CPPBuilder::build_use (std::ofstream &os, std::string use)
  {
    if (use.compare ("core") == 0)
      return;
    else if (use.compare ("gui") == 0) {
      if (!m_display_initialized) {
        os << "#include \"display/display.h\"\n";
        m_display_initialized = true;
      }
    } else if (use.compare ("display") == 0) {
      if (m_display_initialized)
        return;
      m_display_initialized = true;
    }
    os << "#include \"" << use << "/" << use << ".h\"\n";
  }

  void
  CPPBuilder::build_import (std::ofstream &os, std::string import)
  {
    std::replace (import.begin (), import.end (), '.', '/');
    os << "#include \"" << import << ".h\"\n";
  }

  void
  CPPBuilder::build_step (std::ofstream &os, Node *node, bool is_incr)
  {
    if (!m_in_for)
      indent (os);
    std::pair<std::string, std::vector<std::string>> arg = parse_symbol (node->name ());
    if (arg.first.rfind ("cpnt_", 0) == 0) {
      os << "((AbstractProperty*) ";
      if (!arg.second.empty ())
        print_find_child (os, node, arg);
      else
        os << arg.first;
      os << ")->set_value (";
      os << "((AbstractProperty*) ";
      if (!arg.second.empty ())
        print_find_child (os, node, arg);
      else
        os << arg.first;
      os << ")->get_double_value ()";
      if (is_incr)
        os << " +";
      else
        os << " - ";
      os << " 1, true)";
    } else {
      os << arg.first;
      if (is_incr)
        os << "++";
      else
        os << "--";
    }
    if (!m_in_for)
      os << ";\n";
  }

  void
  CPPBuilder::build_print (std::ofstream &os, Node *node)
  {
    indent (os);
    std::string name ("var_" + std::to_string (m_var_num++));
    os << "std::string " << name << " ( ";
    for (auto cur: node->get_expression()) {
      TermNode *n = static_cast<TermNode*> (cur);
      switch (n->arg_type ()) {
        case SYMBOL: {
          if (n->arg_value() != "+")
            print_error_message (error_level::error, "only + symbol is allowed in string expression", 1);
          os << " " << n->arg_value () << " ";
        }
          break;
        case VALUE: {
          os << n->arg_value ();
        }
          break;
        case STRING_VALUE:
          os << n->arg_value ();
          break;
        case VAR: {
          std::pair<std::string, std::vector<std::string>> p = parse_symbol (n->arg_value ());
          // if the name contains "var_" then this is a simple variable not a djnn property
          // so write it as is and return
          std::size_t found = p.first.find ("var_");
          if (found != std::string::npos) {
            os << p.first;
            return;
          }
          os << "((AbstractProperty*)";
          if (p.second.empty ()) {
            os << p.first;
          } else {
            print_find_child (os, node, p);
          }
          os << ")->get_string_value ()";
          break;
        }
        default:
          return;
      }
    }
    os << ");\n";
    indent (os);
    os << "cout << " << name << ";\n";
  }

  void
  CPPBuilder::build_while (std::ofstream &os, Node *node)
  {
    indent (os);
    os << "while (";
    m_in_static_expr = true;
    for (auto cur: node->get_expression()) {
      build_node (os, cur);
    }
    m_in_static_expr = false;
    os << ") {\n";
    m_indent++;
  }

  void
  CPPBuilder::build_for (std::ofstream &os, Node *node)
  {
    ForNode* n = (ForNode*) node;
    indent (os);
    os << "for ";
    m_in_for = true;
  }

  void
  CPPBuilder::build_control_node (std::ofstream &os, Node *node)
  {
    CtrlNode *ctrl = static_cast<CtrlNode*> (node);
    if (node->djnn_type ().compare ("FSMTransition") == 0) {
      build_transition_node (os, ctrl);
      return;
    }
    std::string constructor = get_constructor (node->djnn_type ());
    std::pair<std::string, std::vector<std::string>> src, dst;
    bool is_binding = node->djnn_type ().compare ("Binding") == 0;

    switch (ctrl->in ()->node_type ()) {
      case BINARY_OP: {
        src.first = ctrl->in ()->build_name ();
        src.second.push_back ( "result");
        break;
      }
      case UNARY_OP: {
        src.first = ctrl->in ()->build_name ();
        src.second.push_back ("output");
        break;
      }
      case LOCAL_NODE: {
        LocalNode *n = static_cast<LocalNode*> (ctrl->in ());
        src.first = n->root ()->build_name ();
        std::string path = n->path ();
        std::replace (path.begin (), path.end (), '.', '/');
        src.second.push_back (path);
        break;
      }
      case LITERAL: {
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        indent (os);
        print_start_component (os, new_name, get_constructor ("Double"));
        os << "(" << node->parent ()->build_name () << ", " << m_null_string
            << ", " << ctrl->in ()->name () << ");\n";
        src.first = new_name;
        //src.second.push_back (m_null_string);
        break;
      }
      case TEXT: {
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        indent (os);
        print_start_component (os, new_name, get_constructor ("String"));
        os << "(" << node->parent ()->build_name () << ", " << m_null_string
            << ", " << ctrl->in ()->name () << ");\n";
        src.first = new_name;
        //src.second.push_back (m_null_string);
        break;
      }
      default:
        if (!ctrl->in ()->build_name ().empty ()) {
          src.first = ctrl->in ()->build_name ();
          //src.second.push_back (m_null_string);
        } else if (!ctrl->in ()->name ().empty ()) {
          src = parse_symbol (ctrl->in ()->name ());
          std::string prefix = "var_";
          bool _is_var = src.first.substr (0, prefix.size ()) == prefix;
          if (_is_var) {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            indent (os);
            print_start_component (os, new_name, get_constructor ("Double"));
            os << "(" << node->parent ()->build_name () << ", " << m_null_string
                << ", " << src.first << ");\n";
            src.first = new_name;
            //src.second.push_back (m_null_string);
          }
        } else {
          print_error_message (error_level::warning,
                               "anonymous component in input of control node",
                               1);
          m_error |= 1;
        }
    }

    switch (ctrl->out ()->node_type ()) {
      case LOCAL_NODE: {
        LocalNode *n = static_cast<LocalNode*> (ctrl->out ());
        dst.first = n->root ()->build_name ();
        std::string path = n->path ();
        std::replace (path.begin (), path.end (), '.', '/');
        dst.second.push_back (path);
        break;
      }
      default:
        if (!ctrl->out ()->build_name ().empty ()) {
          dst.first = ctrl->out ()->build_name ();
          //dst.second.push_back (m_null_string);
        } else if (!ctrl->out ()->name ().empty ()) {
          dst = parse_symbol (ctrl->out ()->name ());
        } else {
          print_error_message (error_level::error,
                               "anonymous component in output of control node",
                               1);
        }
    }

    if (is_binding) {
      /* check for synchronizer */
      indent (os);
      os << "if (dynamic_cast<Synchronizer*> (" << build_find_child (node, dst) << ") != nullptr) {\n";
      indent (os);
      os << "\t((Synchronizer*) " << build_find_child (node, dst) << ")->add_source (" << build_find_child (node, src) << ", \"\");\n";
      indent (os);
      os << "} else {\n";
      m_indent++;
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
      os << ", " << build_find_child (node, src) << ", \"\", ";
      os << ctrl->get_in_act () << ", " << build_find_child (node, dst) << ", \"\""
          << ", " << ctrl->get_out_act () << ");\n";
      m_indent--;
      indent (os);
      os << "}\n";
    } else {
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
      os << ", " << build_find_child (node, src) << ", \"\"" << ", ";
      os << build_find_child (node, dst) << ", \"\"";
      if (node->djnn_type ().compare ("Assignment") == 0
          || node->djnn_type ().compare ("PausedAssignment") == 0) {
        os << ", " << node->args ().at (0).second;
      }
      os << ");\n";
    }
  }

  void
  CPPBuilder::build_simple_control_node (std::ofstream &os,
                                         NativeExpressionNode *node)
  {
    std::string p_name =
        node->parent () == nullptr ? "nullptr" : node->parent ()->build_name ();
    TermNode* arg_node = node->get_expression ().at (0);
    std::pair<std::string, std::vector<std::string>> arg;
    if (arg_node->arg_type () != VAR) {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      indent (os);
      if (arg_node->arg_value ().at (0) == '\"') {
        os << "TextProperty *" << new_name << " = new TextProperty (" << p_name
            << ", \"\", " << arg_node->arg_value () << ");\n";
      } else {
        os << "DoubleProperty *" << new_name << " = new DoubleProperty ("
            << p_name << ", \"\", " << arg_node->arg_value () << ");\n";
      }
      std::vector<std::string> children;
      arg = std::pair<std::string, std::vector<std::string>> (new_name, children);
    } else {
      arg = parse_symbol (arg_node->arg_value ());
    }
    if (!node->is_connector () && !node->name ().empty ()) {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      std::pair<std::string, std::vector<std::string> >out_arg = parse_symbol (
          node->get_output_nodes ().at (0));
      os << "Process *" << new_name << " = new Assignment ( " << p_name
          << ", \"\", " << build_find_child (node, arg) << ", \"\","
          << build_find_child (node, out_arg) << ", \"\", "
          << node->is_model () << ");\n";
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    } else {
      for (auto e : node->get_output_nodes ()) {
        indent (os);
        std::pair<std::string, std::vector<std::string> > out_arg = parse_symbol (e);
        os << "new ";
        if (node->is_paused ())
          os << "Paused";
        if (node->is_connector ())
          os << "Connector (";
        else
          os << "Assignment (";
        os << p_name << ", \"\", " << build_root_and_path (arg) << ","
                                   << build_root_and_path (out_arg);
        // connectors don't have is_model but copy_on_activation so the meaning of this property is somewhat inverted
        if (node->is_connector())
          os << ", " << !node->is_model () << ");\n";
        else
          os << ", " << node->is_model () << ");\n";
      }
    }
  }


  string
  transform_name(const string& input)
  {
    string new_param_name = input;
    replace(new_param_name.begin(), new_param_name.end(), '.','_');
    replace(new_param_name.begin(), new_param_name.end(), '/','_');
    replace(new_param_name.begin(), new_param_name.end(), '-','_');
    new_param_name.erase(std::remove(new_param_name.begin(), new_param_name.end(), '"'), new_param_name.end());
    return new_param_name;
  }


  void
  CPPBuilder::build_native_expression_node (std::ofstream &os, Node *n)
  {
    NativeExpressionNode *node = static_cast<NativeExpressionNode*> (n);
    if (node->get_expression ().size () == 1) {
      build_simple_control_node (os, node);
      return;
    }
    std::map<std::string, std::string> sym;
    std::string sym_name ("sym_" + std::to_string (m_sym_num++));
    std::string p_name =
        node->parent () == nullptr ? "nullptr" : node->parent ()->build_name ();
    std::vector<std::string> triggers;
    indent (os);


    string native_name_struct = node->get_build_name () + "_struct";
    string native_name_obj = node->get_build_name () + "_obj";


    std::string native_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::string n_expr_name =
        node->name ().empty () ? m_null_string : node->name ();
    os << "auto *" << native_name << " = new "<< native_name_struct << " ("
       << p_name << ", " << n_expr_name << ", true, " << node->is_model ()
       << ");\n";

    for (auto e : node->get_expression ()) {
      if ((e->arg_type () == VAR
          || e->arg_type () == CAST_STRING
          || e->arg_type () == CAST_DOUBLE
          || e->arg_type () == CAST_PROCESS)
          && sym.find (e->arg_value ()) == sym.end ()) {
        std::pair<std::string, std::vector <std::string>> arg = parse_symbol (
            e->arg_value ());

        if (arg.first.compare (0, 6, "d_var_") == 0
            || arg.first.compare (0, 6, "i_var_") == 0) {
          indent (os);
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "DoubleProperty *" << new_name << " = new DoubleProperty ("
              << p_name << ", \"\", " << arg.first << ");\n";
          indent (os);
          string new_param_name = transform_name(e->arg_value ());
            os << native_name << "->"<< new_param_name <<  "= " << new_name
                << ";\n";

        } else if (arg.first.compare (0, 6, "s_var_") == 0) {
          indent (os);
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "TextProperty *" << new_name << " = new TextProperty ("
              << p_name << ", \"\", " <<arg.first << ");\n";
          indent (os);
          string new_param_name = transform_name(e->arg_value ());
            os << native_name << "->"<< new_param_name <<  "= " << new_name
                << ";\n";

        } else {
          indent (os);

          if (!arg.second.empty ()) {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            os << "AbstractProperty* " << new_name
                << " = dynamic_cast<AbstractProperty*> (" << build_find_child (node, arg) << ");\n";
            indent (os);
            os << "if (" << new_name << " == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e->arg_value ()
                << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            string new_param_name = transform_name(e->arg_value ());
            os << native_name << "->"<< new_param_name <<  "= " << new_name
                << ";\n";
            sym[e->arg_value ()] = new_name;
            triggers.push_back (new_name);

          } else {
            os << "if (dynamic_cast<AbstractProperty*>(" << arg.first
                << ") == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e->arg_value ()
                << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            string new_param_name = transform_name(e->arg_value ());
            os << native_name << "->"<< new_param_name <<  " = dynamic_cast<AbstractProperty*>(" << arg.first
                << ");\n";
            sym[e->arg_value ()] = arg.first;
            triggers.push_back (arg.first);
            
          }
        }
      }
    }

    for (auto e : node->get_output_nodes ()) {
      if (sym.find (e) == sym.end ()) {
        std::pair<std::string, std::vector<std::string>> arg = parse_symbol (e);
        if (arg.first.compare (0, 4, "var_") != 0) {
          indent (os);
          if (arg.second.empty ()) {
            os << "if (dynamic_cast<AbstractProperty*>(" << arg.first
                << ") == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            string new_param_name = transform_name(e);
            os << native_name << "->"<< new_param_name
                << " = dynamic_cast<AbstractProperty*>(" << arg.first
                << ");\n";
            sym[e] = arg.first;
          } else {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            os << "AbstractProperty* " << new_name
                << " = dynamic_cast<AbstractProperty*> (" << build_find_child (node, arg) << ");\n";
            indent (os);
            os << "if (" << new_name << " == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            string new_param_name = transform_name(e);
            os << native_name << "->" << new_param_name << " = " << new_name << ";\n";
            sym[e] = new_name;
          }
        }
      }
    }

    indent (os);
    // now that properties are built, call finalize_construction, wich in turn will call impl_activate
    os << native_name << "->finalize_construction (" << p_name << ", " << n_expr_name << ");\n";

    std::string& new_name = native_name;

    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    std::string native_edge_name = new_name;
    if (node->is_connector ()) {
      indent (os);
      std::string sync_name ("cpnt_" + std::to_string (m_cpnt_num++));
      os << "Synchronizer* " << sync_name << " = new Synchronizer (" << p_name
          << ", \"sync_"+sync_name+"\", " << new_name << ", \"\");\n";
      native_edge_name = sync_name;
      for (auto t : triggers) {
        indent (os);
        os << sync_name << "->add_source (" << t << ", \"\");\n";
      }
    }
    for (auto out : node->get_output_nodes ()) {
      std::pair<std::string, std::vector <std::string>> arg = parse_symbol (out);
      indent (os);
      os << native_edge_name <<"->add_native_edge (" << new_name << "," ;
      if (!arg.second.empty ())
        os << build_find_child (node, arg);
      else
        os << arg.first;
      os << ");\n";
    }
  }

  void
  CPPBuilder::build_activator (std::ofstream &os, ActivatorNode *n)
  {
    indent (os);
    std::string p_name =
        n->parent () == nullptr ? "nullptr" : n->parent ()->build_name ();
    os << "new Activator (" << p_name << ", \"\", " << n->node ()->build_name ()
        << "->find_child (\"action\"));\n";
  }

  void
  CPPBuilder::build_native_action (std::ofstream &os, Node *n)
  {
    NativeActionNode *node = static_cast<NativeActionNode*> (n);
    os << "void\n";
    os << node->action_name () << "(Process *" << node->param_name () << ")\n";
    const std::string code = node->code ();
    if (code[0] != '{')
      os << "{\n";
    os << code;
    if (code[code.length () - 1] != '}')
      os << "}";
    os << std::endl;
  }

  void
  CPPBuilder::build_native_expression (std::ofstream &os, Node *n)
  {
    NativeExpressionNode *node = static_cast<NativeExpressionNode*> (n);
    if (node->get_expression ().size () == 1) {
      return;
    }
    std::string unique_name = m_filename;
    std::replace (unique_name.begin (), unique_name.end (), '/', '_');
    std::replace (unique_name.begin (), unique_name.end (), '.', '_');

    std::string native_name ("nat_" + unique_name + "_" + std::to_string (m_native_num++));
    node->set_build_name (native_name);

    // first pass: generate nat struct
    map<string,bool> already_handled;
    string native_name_struct = native_name + "_struct";
    string native_name_obj = native_name + "_obj";

    os << "\n";
    os << "struct " << native_name_struct << " : public NativeExpressionAction {\n";
    os << "\t" << native_name_struct << R"( (Process *p, const std::string &n, bool string_setter, bool isModel)
      : NativeExpressionAction (p, n, isModel), _string_setter (string_setter)
    {
      set_is_model (isModel);

      // note: finalize_construction will call impl_activate before proper properties are fully built, leading to a crash
      // so delay finalize_construction after building properties
      // Process::finalize_construction (p, n);
    }
    // make finalize_construction public
    void finalize_construction (Process* p, const string& n) {
       Process::finalize_construction (p, n);
    }
    bool _string_setter;
    void impl_deactivate () override {}
    void impl_activate () override;)" << "\n";

    for (auto n : node->get_output_nodes ()) {
      string tn = transform_name(n);
      if(already_handled.count(tn)==0) {
        os << "\tAbstractProperty * " << tn << ";\n";
        already_handled[tn] = true;
      }

      for (auto op : node->get_expression ()) {
        if (op->arg_type () == VAR
              || op->arg_type () == CAST_DOUBLE) {
            string tn = transform_name(op->arg_value ());
            if(already_handled.count(tn)==0) {
              os << "\tAbstractProperty * " << tn << ";\n";
              already_handled[tn] = true;
            }
          } else if (op->arg_type () == CAST_STRING) {
            string tn = transform_name(op->arg_value ());
            if(already_handled.count(tn)==0) {
              os << "\tAbstractProperty * " << tn << ";\n";
              already_handled[tn] = true;
            }
          } else if (op->arg_type () == CAST_PROCESS) {
            string tn = transform_name(op->arg_value ());
            if(already_handled.count(tn)==0) {
              os << "\tAbstractProperty * " << tn << ";\n";
              already_handled[tn] = true;
            }
          } else if (op->arg_value ().size() >= 1 && op->arg_value ().at (0) == '\"') {
          } else {
          }
      }
    }
    os << "};\n";


    // second pass: generate nat

    os << "\nvoid\n" << native_name_struct << "::impl_activate ()\n{\n";
    for (auto n : node->get_output_nodes ()) {
      os << "\t" << transform_name (n) << "->set_value(";
      for (auto op : node->get_expression ()) {
        if (op->arg_type () == VAR || (op->arg_type () == CAST_DOUBLE)) {
          string tn = transform_name (op->arg_value ());
          os << tn << "->get_double_value()";
        } else if (op->arg_type () == CAST_STRING) {
          string tn = transform_name (op->arg_value ());
          os << tn << "->get_string_value()";
        } else if (op->arg_type () == CAST_PROCESS) {
          string tn = transform_name (op->arg_value ());
          os << tn;
        } else if (op->arg_value ().size () >= 1
            && op->arg_value ().at (0) == '\"') {
          os << "std::string (" << op->arg_value () << ")";
        } else {
          os << op->arg_value ();
        }
      }
      os << ", " << !node->is_paused () << ");\n";
    }
    indent (os);
    os << "};\n\n";

  }

  void
  CPPBuilder::build_end_define (std::ofstream &os, Node *node)
  {
    indent (os);
    os << "return " << m_parent_list.back ()->get_symbol ("this") << ";\n}\n";
    m_indent--;
    BuildNode* n = m_parent_list.at (m_parent_list.size () - 1);
    m_parent_list.pop_back ();
    if (n)
      delete n;
    n = m_parent_list.at (m_parent_list.size () - 1);
    m_parent_list.pop_back ();
    if (n)
      delete n;
  }

  void
  CPPBuilder::build_instruction (std::ofstream &os, Node *node)
  {
    InstructionNode *n = static_cast<InstructionNode*> (node);
    for (int i = 0; i < n->cpnt_list ().size (); i++) {
      std::pair<std::string, std::vector <std::string>> arg = parse_symbol (
        n->cpnt_list ().at (i));
      std::string cpnt_name =
      arg.second.empty () ?
      arg.first : build_find_child (node, arg);
      if (arg.first.empty ()) {
        print_error_message (error_level::error,
         "unknown component " + n->cpnt_list ().at (i), 1);
        return;
      }
      indent (os);
      switch (n->type ()) {
        case DUMP:
        os << "if (" << cpnt_name << ")" << endl ;
        indent (os); indent (os);
        os << cpnt_name << "->dump";
        if (!n->has_argument ()) {
          os << " (0);\n";
          indent (os);
          os << "else" << endl ;
          indent (os); indent (os);
          os << "cout <<  endl << endl << \"warning - dump could not resolve: \" << " << cpnt_name <<  " << endl << endl;" << endl;
       }
       else {
        os << " (";
        for (auto arg : n->args ()) {
          build_term_node (os, arg);
        }
        os << ";\n";
      }
      break;
      case XMLSERIALIZE:
        os << "if (" << cpnt_name << ")" << endl ;
        indent (os); indent (os);
        os << cpnt_name << "->serialize (\"XML\");\n";
        indent (os);
        os << "else" << endl ;
        indent (os); indent (os);
        os << "cout <<  endl << endl << \"warning - XMLSerialize could not resolve: \" << " << cpnt_name <<  " << endl << endl;" << endl;
      break;
      case NOTIFY:
      os << cpnt_name << "->notify_activate ();\n";
      break;
      case RUN:
      if (n->cpnt_list ().at (i).compare ("syshook") == 0) {
        if (n->has_argument ()) {
          os << "MainLoop::instance ().set_run_for (";
          for (auto arg : n->args ()) {
            build_term_node (os, arg);
          }
        }
        os << "MainLoop::instance ().activate ();\n";
      } else
      os << cpnt_name << "->activate ();\n";
      break;
      case STOP:
      if (cpnt_name.compare ("syshook") == 0) {
        os << "MainLoop::instance ().deactivate ();\n";
      } else
      os << cpnt_name << "->deactivate ();\n";
      break;
      case DELETE:
         /* delete first.second */
      if (!arg.second.empty ()) {
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        os << "Process *" << new_name << " = " << cpnt_name << ";\n";
        indent (os);
        os << "if (" << new_name << ") {\n";
        indent (os); indent (os);
        os << new_name << "->deactivate ();\n";
        indent (os); indent (os);
        os << new_name << "->get_parent ()->remove_child (" << new_name << ");\n";
        indent (os); indent (os);
        os << new_name << "->schedule_delete ();\n";
        indent (os); indent (os);
        os << new_name << " = nullptr;\n";
        indent (os);
        os << "}\n";
      }
        /*  delete first */
      else {
        os << "if (" << arg.first << ") {\n";
        indent (os); indent (os);
        os << arg.first << "->deactivate ();\n";
        indent (os); indent (os);
        os << "if (" << arg.first << "->get_parent ())\n";
        indent (os); indent (os);
        indent (os);
        os << arg.first << "->get_parent ()->remove_child (" << arg.first
        << ");\n";
        indent (os); indent (os);
        os << arg.first << "->schedule_delete ();\n";
        indent (os); indent (os);
        os << arg.first << " = nullptr;\n";
        indent (os);
        os << "}\n";
      }
      break;
      case UNKNOWN:
      print_error_message (error_level::error,
       "unknown instruction " + n->cpnt_list ().at (i), 1);
      break;
    }
  }
}

  void
  CPPBuilder::set_property (std::ofstream &os, Node *node)
  {
    if (!m_in_for)
      indent (os);

    // if the symbol is unknown we take it as the definition of a new Process*
    if (!known_symbol (node->name ())) {
      std::string var_name ("cpnt_" + std::to_string (m_cpnt_num++));
      if (m_parent_list.back ()->add_entry (node->name (), var_name) == 1
                && node->duplicate_warning ())
              print_error_message (error_level::warning,
                                   "duplicated name: " + node->name (), 0);
      os << "Process* " << var_name << " = ";
    } else {
      std::pair<std::string, std::vector<std::string>> arg = parse_symbol (node->name ());
      if (arg.first.rfind ("cpnt_", 0) == 0) {
        os << "((AbstractProperty*) ";
        if (!arg.second.empty ())
          print_find_child (os, node, arg);
        else
          os << arg.first;
        os << ")->set_value (";
        m_in_set_property = true;
      } else {
        os << arg.first << " = ";
      }
    }
  }

  void
  CPPBuilder::end_set_property (std::ofstream &os, Node *node)
  {
    if (m_in_set_property) {
      os << ", true)";
      m_in_set_property = false;
    }
 //   if (!m_in_for)
   //   os << ";\n";
  }

  bool
  CPPBuilder::known_symbol (const string& name)
  {
    std::string str;
    std::size_t pos = name.find ('.');
    if (pos == std::string::npos) {
      str = m_parent_list.back ()->get_symbol (name);
      if (str.empty ())
        return false;
    }
    return true;
  }

  void
  CPPBuilder::set_ref_property (std::ofstream &os, Node *node)
  {
    std::string dst = node->args ()[0].second;
    std::string src = node->args ()[1].second;
    indent (os);
    std::pair<std::string, std::vector<std::string>> arg = parse_symbol (dst);
    os << "((RefProperty*) ";
    if (!arg.second.empty ())
      print_find_child (os, node, arg);
    else
      os << arg.first;
    os << ")->set_value (";
    std::pair<std::string, std::vector<std::string>> arg2 = parse_symbol (src);
    if (!arg2.second.empty ())
      print_find_child (os, node, arg2);
    else
      os << arg2.first;
    os << ", true);\n";
  }

  void
  CPPBuilder::end_property (std::ofstream &os, Node *node)
  {
    if (node->name ().compare ("Text") == 0)
      os << ")";
    os << ", true);\n";
    m_in_set_text = false;
  }

  void
  CPPBuilder::build_term_node (std::ofstream &os, Node *node)
  {
    TermNode *n = static_cast<TermNode*> (node);
    switch (n->arg_type ()) {
      case SYMBOL: {
        if (m_in_set_text) {
          os << ")";
        }
        os << n->arg_value ();
        if (m_in_set_text) {
          os << "std::string (";
        }
      }
        break;
      case VALUE: {
        if (m_in_set_text) {
          os << "to_string (";
        }
        os << n->arg_value ();
        if (m_in_set_text) {
          os << ")";
        }
      }
        break;
      case FUNCTION_CALL:
      case STRING_VALUE:
        os << n->arg_value ();
        break;
      case VAR: {
        if (m_in_switch) {
          os << "\"" << n->arg_value () << "\"";
          m_in_switch = false;
          return;
        }
        std::pair<std::string, std::vector<std::string>> p = parse_symbol (n->arg_value ());
        // if the name contains "var_" then this is a simple variable not a djnn property
        // so write it as is and return
        std::size_t found = p.first.find ("var_");
        if (found != std::string::npos) {
          os << p.first;
          return;
        }

        if (p.second.empty ())
          os << p.first;
        else
          print_find_child (os, node, p);

        break;
      }
      case CAST_PROCESS: {
        std::pair<std::string, std::vector<std::string>> p = parse_symbol (n->arg_value ());
        if (p.second.empty ())
          os << p.first;
        else
          print_find_child (os, node, p);

        break;
      }
      case CAST_DOUBLE: {
        std::pair<std::string, std::vector<std::string>> p = parse_symbol (n->arg_value ());
        std::size_t found = p.first.find ("var_");
        if (found != std::string::npos) {
          os << p.first;
          return;
        }
        os << "((AbstractProperty*)";
        if (p.second.empty ())
          os << p.first;
        else
          print_find_child (os, node, p);
        os << ")->get_double_value ()";
        break;
      }
      case SMALA_NULL: {
        os << m_null_symbol;
        break;
      }
      case START_LCB_BLOCK: {
        os << "\n";
        indent (os);
        os << "{\n";
        m_indent++;
        break;
      }
      case END_LCB_BLOCK: {
        os << "\n";
        m_indent--;
        indent (os);
        os << "}\n";
        break;
      }
      case CAST_STRING: {
        std::pair<std::string, std::vector<std::string>> p = parse_symbol (n->arg_value ());
        os << "((AbstractProperty*)";
        if (p.second.empty ())
          os << p.first;
        else
          print_find_child (os, node, p);
        os << ")->get_string_value ()";
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
  CPPBuilder::build_set_string (std::ofstream &os, const std::string &cpnt_name,
                                const std::string &spec,
                                const std::string &value)
  {
    os << "((TextProperty*) " << cpnt_name << "->find_child (" << spec
        << "))->set_value (string (" << value << "), true);\n";
  }

  void
  CPPBuilder::get_property (std::ofstream &os, Node *node)
  {
    std::string var_name;
    if (node->djnn_type ().compare ("GetRef") == 0)
      var_name = string ("cpnt_" + std::to_string (m_cpnt_num++));
    else
      var_name = ("pr_var_" + std::to_string (m_var_num++));
    if (m_parent_list.back ()->add_entry (node->name (), var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);

    if (node->djnn_type ().compare ("doubleToString") == 0) {
      os << "std::string " << var_name
          << " = std::to_string (((DoubleProperty*) ";
    } else if (node->djnn_type ().compare ("intToString") == 0) {
      os << "std::string " << var_name << " = std::to_string (((IntProperty*) ";
    } else {
      print_type (os, node->args ().at (0).first);
      os << " " << var_name << " = ";
      os << "((" << node->djnn_type ().substr (3) << "Property*) ";
    }
    std::pair<std::string, std::vector<std::string>> arg = parse_symbol (
        node->args ().at (0).second);
    if (!arg.second.empty ())
      print_find_child (os, node, arg);
    else
      os << arg.first;
    os << ")->get_value ()";
    if (node->djnn_type ().compare ("doubleToString") == 0
        || node->djnn_type ().compare ("intToString") == 0) {
      os << ")";
    }
    os << ";\n";
  }

  void
  CPPBuilder::alias (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "alias (" << m_parent_list.back ()->name () << ", \""
        << n->left_arg () << "\", ";
    std::pair<std::string, std::vector <std::string>> arg = parse_symbol (n->right_arg ());
    if (arg.second.empty ())
      os << arg.first << ");\n";
    else
      os << build_find_child (n, arg) << ");\n";
    indent (os);
    os << "Process *" << new_name << " = " << m_parent_list.back ()->name ()
        << "->find_child ( \"" << n->left_arg () + "\");\n";
    if (m_parent_list.back ()->add_entry (n->left_arg (), new_name) == 1
        && node->duplicate_warning ())
      print_error_message (error_level::warning,
                           "duplicated name: " + n->left_arg (), 0);
  }

  void
  CPPBuilder::merge (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::vector<std::string>> left = parse_symbol (n->left_arg ());
    std::pair<std::string, std::vector<std::string>> right = parse_symbol (n->right_arg ());
    os << "merge_children (" << build_find_child (node, left) << ", \"\", "
        << build_find_child (node, right) << ", \"\");\n";
  }

  void
  CPPBuilder::remove (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::vector<std::string>> right = parse_symbol (n->left_arg ());
    std::pair<std::string, std::vector<std::string>> left = parse_symbol (n->right_arg ());
    os << left.first << "->remove_child ( " << right.first << ");\n";
  }

  void
  CPPBuilder::move (std::ofstream &os, Node *node, const string &c)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::string last;
    std::pair<std::string, std::vector<std::string>> left = parse_symbol (n->left_arg ());
    if (!n->right_arg ().empty()) {
      std::pair<std::string, std::vector<std::string>> last = parse_symbol (n->right_arg ());
      os << left.first << "->get_parent ()->move_child (" << left.first << ", "
              << c << ", " << build_find_child (node, last) << ");\n";
    }
    else {
      os << left.first << "->get_parent ()->move_child (" << left.first << ", "
        << c << ", nullptr);\n";
    }
  }

  void
  CPPBuilder::repeat (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);

    // TODO we should also parse properly argument expressions

    indent (os);
    os << "Process *" << new_name << " = new List ("
        << m_parent_list.back ()->name () << ", \"" << node->name ()
        << "\");\n";
    indent (os);
    std::string new_var_name ("var_" + std::to_string (m_var_num++));
    os << "int " << new_var_name << ";\n";
    if (m_parent_list.back ()->add_entry (node->args ().at (0).second,
                                          new_var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->args ().at (0).second,
                           0);

    indent (os);
    string nb_entries =
        node->args ().at (1).first == INT ?
            node->args ().at (1).second :
            m_parent_list.back ()->get_symbol (node->args ().at (1).second);
    os << "for (" << new_var_name << " = 1; " << new_var_name << " <= "
        << nb_entries << "; " << new_var_name << "++) {\n";
    m_indent++;
    std::string inFor ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "Process *" << inFor << " = new Component (" << new_name
        << ", \"\");\n";
    m_parent_list.push_back (new BuildNode (inFor, m_parent_list.back ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
    node->set_build_name (inFor);
  }

  void
  CPPBuilder::load_xml (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = XML::djnLoadFromXML (";
    if (arg.first == STRING) {
      os << arg.second;
    } else {
      os << m_parent_list.back ()->get_symbol (arg.second);
    }
    os << ");\n";
  }

  void
  CPPBuilder::add_child (std::ofstream &os, Node *node)
  {
    std::pair<std::string, std::vector<std::string>> s = parse_symbol (
        node->args ().at (0).second);
    indent (os);
    if (s.second.empty ()) {
      m_parent_list.back ()->add_entry (node->name (), s.first);
      os << m_parent_list.back ()->name () << "->add_child (" << s.first
          << ", \"" << node->name () << "\");\n";
    } else {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1) {
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
      }
      os << "Process *" << new_name << " = " << build_find_child (node, s) << ";\n";
      indent (os);
      os << m_parent_list.back ()->name () << "->add_child (" << new_name
          << ", \"" << node->name () << "\");\n";
    }
  }

  void
  CPPBuilder::fetch_add_child (std::ofstream &os, const std::string &parent,
                               const std::string &child, const std::string &name)
  {
    if (parent =="nullptr")
      return;
    indent (os);
    os << parent << "->add_child (" << child << ", \"" << name << "\");\n";
  }

  void
  CPPBuilder::add_children_to (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::pair<std::string, std::vector <std::string>> s = parse_symbol (node->name ());

    if (!s.second.empty ()) {
      indent (os);
      os << "Process *" << new_name << " = " << build_find_child (node, s) << ";\n";
      m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (new_name);
      indent (os);
      os << "if (" << new_name << " == nullptr)\n";
      indent (os);
      os << "\tcerr <<  endl << endl << \"ERROR - processing addChildrenTo - the component \\\"\" << \"" << node->name () << "\\\"\" << \" is null or do not exist yet\" << endl << endl;\n";
    } else {
      m_parent_list.push_back (new BuildNode (s.first, m_parent_list.back ()));
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (s.first);
    }
  }

  void
  CPPBuilder::find (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = ";
    if (arg.first == STRING) {
      os << "Process::find_child (nullptr, " << arg.second;
    } else {
      if (node->args ().size () == 2) {
        string root = m_parent_list.back ()->get_symbol (arg.second);
        os << root << "->find_child (" << node->args ().at (1).second << ")";
      } else {
        std::pair<std::string, std::vector<std::string>> p = parse_symbol (arg.second);
        print_find_child (os, node, p);
      }
    }
    os << ";\n";
  }

  void
  CPPBuilder::clone (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = ";
    std::pair<std::string, std::vector<std::string>> p = parse_symbol (arg.second);
    if (p.second.empty ())
      os << p.first << "->clone ();\n";
    else
      os << build_find_child (node, p) << "->clone();\n";
  }

  void
  CPPBuilder::build_this_node (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    os << "Process* " << new_name << " = new Component  (p, n);\n";

    /* We make the hypothesis that "this" is the first node after _define_ thus
     * the symbols in the sym_table should be the arguments of the function.
     * They must be duplicated to check for symbol duplication */
    BuildNode *new_parent = new BuildNode (new_name, m_parent_list.back ());
    new_parent->sym_table ()->insert (
        m_parent_list.back ()->sym_table ()->begin (),
        m_parent_list.back ()->sym_table ()->end ());
    m_parent_list.push_back (new_parent);
  }

  void
  CPPBuilder::build_define_node (std::ofstream &os, Node *node)
  {
    m_parent_list.push_back (new BuildNode ("", m_parent_list.back ()));
    os << "Process*\n" << node->name () << " (Process *p, const string &n";
    for (int j = 0; j < node->args ().size (); j++) {
      std::pair<ParamType, std::string> arg = node->args ().at (j);
      os << ", ";
      print_type (os, arg.first);
      std::string new_name;
      switch (arg.first) {
        case INT:
          new_name = "i_var_" + std::to_string (m_var_num++);
          break;
        case DOUBLE:
          new_name = "d_var_" + std::to_string (m_var_num++);
          break;
        case STRING:
          new_name = "s_var_" + std::to_string (m_var_num++);
          break;
        case LOCAL_NAME:
        case NAME:
        case PROCESS:
          new_name = "cpnt_" + std::to_string (m_cpnt_num++);
          break;
        default:;
      }
      os << " " << new_name;
      if (m_parent_list.back ()->add_entry (arg.second, new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + arg.second, 0);
    }
    os << ")\n{\n";
    m_indent++;
  }

  void
  CPPBuilder::build_main_node (std::ofstream &os)
  {

    /* main */
    os << "int\nmain () {\n";
    m_indent = 1;
    int size = m_ast.preamble ().use ().size ();
    bool has_display = false;
    /* init modules from use */
    for (int i = 0; i < size; ++i) {
      std::string str = m_ast.preamble ().use ().at (i);
      if (str == "display") {
        if (!has_display) {
          has_display = true;
        }
        else {
          continue;
        }
      }
      if (str == "gui" && !has_display) {
        indent (os);
        os << "init_display ();\n";
        has_display = true;
      }
      if (str == "core") {
        indent (os);
        os << "init_exec_env ();\n";
      }
      /* add cpp init_MODULE corresponding */
      indent (os);
      os << "init_" << str << " ();\n";

    }
  }

  void
  CPPBuilder::build_end_main (std::ofstream &os, Node *node)
  {
    if (m_ast.get_root_node() == nullptr)
      return;
    indent (os);
    os << m_ast.get_root_node ()->build_name () << "->activate ();\n";
    indent (os);
    os << "MainLoop::instance ().activate ();\n";
  }

  void
  CPPBuilder::build_native_action_component (std::ofstream &os, Node *node)
  {
    std::string constructor = get_constructor (node->djnn_type ());
    std::string name =
        node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    indent (os);
    std::string p_name =
        node->parent () == nullptr ? "nullptr" : node->parent ()->build_name ();
    os << "Process* " << new_name << " = new " << constructor << " (" << p_name
        << ", " << name << ", " << node->args ().at (0).second << ", ";
    std::string data = node->args ().at (1).second;
    if (data.compare ("0") == 0)
      data = "nullptr";
    else {
      std::pair<std::string, std::vector<std::string>> p = parse_symbol (data);
      if (!p.second.empty ()) {
        data = build_find_child (node, p);
      } else
        data = p.first;
    }
    os << data << ", " << node->args ().at (2).second << ");\n ";
  }

  void
  CPPBuilder::build_transition_node (std::ofstream &os, CtrlNode *ctrl)
  {
    std::string constructor = get_constructor (ctrl->djnn_type ());
    indent (os);
    os << "new " << constructor << " (" << parse_symbol (ctrl->parent ()->name()).first
        << ", " << m_null_string;

    std::pair<std::string, std::vector<std::string> >src, dst;
    src = parse_symbol (ctrl->in ()->name ());
    dst = parse_symbol (ctrl->out ()->name ());
    std::string src_str =
        src.second.empty () ?
            src.first : build_find_child (ctrl, src);
    std::string dst_str =
        dst.second.empty () ?
            dst.first : build_find_child (ctrl, dst);
    os << ", " << src_str << ", " << dst_str << ", ";
    std::pair<std::string, std::vector<std::string>> trigger = parse_symbol (
        ctrl->args ().at (0).second);
    os << build_find_child (ctrl, trigger) << ", \"\", ";
    if (ctrl->args ().size () == 2) {
      std::pair<std::string, std::vector<std::string>> act = parse_symbol (
          ctrl->args ().at (1).second);
      os << build_find_child (ctrl, act) << ", \"\");\n";
    } else {
      os << "nullptr, \"\");\n";
    }
  }

  void
  CPPBuilder::build_binary_node (std::ofstream &os, Node *node)
  {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *left = op->left ();
    Node *right = op->right ();
    std::string prefix = "var_";
    std::string left_sym = m_parent_list.back ()->get_symbol (left->name ());
    std::string right_sym = m_parent_list.back ()->get_symbol (right->name ());
    bool left_is_var = (left_sym.substr (0, prefix.size ())) == prefix;
    if (left_is_var) {
      left->set_name (left_sym);
    }
    bool right_is_var = right_sym.substr (0, prefix.size ()) == prefix;
    if (right_is_var) {
      right->set_name (right_sym);
    }
    std::string vleft =
        left->node_type () == LITERAL || left_is_var ? left->name () : "0";
    std::string vright =
        right->node_type () == LITERAL || right_is_var ? right->name () : "0";
    std::string constructor = get_constructor (node->djnn_type ());

    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ()->add_entry (new_name, new_name);
    node->set_build_name (new_name);
    indent (os);
    os << "Process *" << new_name << " = new " << constructor << " ("
        << m_parent_list.back ()->name () << ", \"\", " << vleft << ", "
        << vright << ");\n";
    if (!left_is_var)
      check_and_build_connector (os, left, new_name, "\"left\"");
    if (!right_is_var)
      check_and_build_connector (os, right, new_name, "\"right\"");
    if (node->in_expression ()) {
      indent (os);
      os << "new Activator (" << m_parent_list.back ()->name () << ", \"\", "
          << new_name << ", \"action\");\n";
    }
  }

  void
  CPPBuilder::build_unary_node (std::ofstream &os, Node *node)
  {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *right = op->right ();
    std::string prefix = "var_";
    std::string right_sym = m_parent_list.back ()->get_symbol (right->name ());
    bool right_is_var = right_sym.substr (0, prefix.size ()) == prefix;
    std::string vright =
        right->node_type () == LITERAL || right_is_var ? right->name () : "0";
    std::string constructor = get_constructor (node->djnn_type ());

    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ()->add_entry (new_name, new_name);
    node->set_build_name (new_name);
    indent (os);
    os << "Process *" << new_name << " = new " << constructor << " ("
        << m_parent_list.back ()->name () << ", \"\", " << vright << ");\n";
    if (!right_is_var)
      check_and_build_connector (os, right, new_name, "\"input\"");
    if (node->in_expression ()) {
      indent (os);
      os << "new Activator (" << m_parent_list.back ()->name () << ", \"\", "
          << new_name << ", \"action\");\n";
    }
  }

  void
  CPPBuilder::build_dash_array (std::ofstream &os, DashArrayNode *node)
  {
    std::string name = node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (node->name ().compare ("_") == 0)
      node->set_name (new_name);
    node->set_build_name (new_name);
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1 && node->duplicate_warning ())
      print_error_message (error_level::warning, "duplicated name: " + node->name (), 0);
    indent (os);
    std::string p_name = node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    os << "DashArray *" << new_name << " =  new DashArray (" << p_name << ", " << name << ");\n";
    int sz = node->get_pattern ().size ();
    if (sz == 0)
      return;
    if (sz == 1) {
      indent (os);
      os << new_name << "->add_sub_pattern (" << node->get_pattern ().at (0) << ", " << node->get_pattern ().at (0) << ");\n";
    } else {
      for (int i = 0; i < sz - 1; i++) {
        indent (os);
        os << new_name << "->add_sub_pattern (" << node->get_pattern ().at (i) << ", " << node->get_pattern ().at (i + 1) << ");\n";
      }
      if (sz % 2 != 0) {
        for (int i = 0; i < sz - 1; i++) {
          indent (os);
          os << new_name << "->add_sub_pattern (" << node->get_pattern ().at (i) << ", " << node->get_pattern ().at (i + 1) << ");\n";
        }
      }
    }
  }

  void
  CPPBuilder::check_and_build_connector (std::ofstream &os, Node *n,
                                         const std::string &name,
                                         const std::string &side)
  {
    std::pair<std::string, std::vector<std::string>> p;
    switch (n->node_type ()) {
      case PATH: {
        p = parse_symbol (n->name ());
        break;
      }
      case BINARY_OP: {
        p = parse_symbol (n->build_name ());
        p.second.push_back ("\"result\"");
        break;
      }
      case UNARY_OP: {
        p = parse_symbol (n->build_name ());
        p.second.push_back ("\"output\"");
        break;
      }
      default:
        return;
    }
    indent (os);
    os << "new Connector (" << m_parent_list.back ()->name () << ", \"\", "
        << build_root_and_path (p) << ", " << name << ", " << side
        << ", false);\n";
  }

  void
  CPPBuilder::build_smala_native (std::ofstream &os, Node *node)
  {
    SmalaNative *n = static_cast<SmalaNative*> (node);
    std::string src_name = "cpnt_" + std::to_string (m_cpnt_num++);
    std::string data_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.push_back (new BuildNode ("0", m_parent_list.back ()));
    os << "\nstatic void\n" << n->fct () << " (Process* c) {\n";
    os << "\tProcess *" << src_name << " = c->get_activation_source ();\n";
    os << "\tProcess *" << data_name
        << " = (Process *) get_native_user_data (c);\n";
    if (m_parent_list.back ()->add_entry (n->src (), src_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->src (), 0);
    if (m_parent_list.back ()->add_entry (n->data (), data_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->data (), 0);
    m_indent = 1;
  }

  void
  CPPBuilder::build_define (const std::string &prefix)
  {
    std::ofstream os (prefix + ".h");
    std::string s = prefix;
    std::replace (s.begin (), s.end (), '/', '_');
    for (int i = 0; i < prefix.length (); i++)
      s.at (i) = std::toupper (s.at (i));
    os << "#pragma once\n#include <string>\n\n";
    for (int i = 0; i < m_ast.define_node_list ().size (); i++) {
      Node *def = m_ast.define_node_list ().at (i);
      os << "djnn::Process* " << def->name ()
          << " (djnn::Process*, const std::string &";
      for (int j = 0; j < def->args ().size (); j++) {
        std::pair<ParamType, std::string> arg = def->args ().at (j);
        os << ", ";
        print_type (os, arg.first);
      }
      os << ");\n";
    }
    os.close ();
  }

  void
  CPPBuilder::print_type (std::ofstream &os, ParamType type)
  {
    switch (type) {
      case INT: {
        os << "int";
        break;
      }
      case DOUBLE: {
        os << "double";
        break;
      }
      case STRING: {
        os << "const std::string&";
        break;
      }
      case NAME:
      case PROCESS:
      {
        os << "djnn::Process*";
        break;
      }
      default:
        break;
    }
  }

  void
  CPPBuilder::print_component_decl (std::ofstream &os, const std::string &name)
  {
    os << "Process* " << name;
  }

  void
  CPPBuilder::print_component_constructor (std::ofstream &os,
                                           const std::string &constructor)
  {
    std::map<std::string, std::string>::iterator it;
    it = m_import_types.find (constructor);
    if (it == m_import_types.end ())
      os << "new " << constructor;
    else
      os << constructor;
  }

  static bool
  is_number(const std::string& s)
  {
      return !s.empty() && std::find_if(s.begin(),
          s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
  }

  std::string
  CPPBuilder::build_find_child (Node *n, const std::pair< std::string, std::vector<std::string> > &sym)
  {
    std::string res = sym.first;
    if (!sym.second.empty ()) {
      res += "->find_child(";
      for (int i = 0; i < sym.second.size (); i++) {
        std::string s = sym.second.at(i);
        if (s[0] == '[') {
          bool is_alone = s[s.length()-1] == ']';
          if (is_alone)
            s = s.substr (1, s.length() - 2);
          bool is_num = is_number (s);
          if (!is_num) {
            s = m_parent_list.back ()->get_symbol (s);
          }
          if (i != 0)
            res += "\")->find_child(";
          std::size_t found = s.find ("var_");
          if (is_alone && (found!=std::string::npos || is_num)) {
            res += s;
            res += ")";
            if (i != sym.second.size ()-1) {
              res += "->find_child(\"";
            } else {
              return res;
            }
          } else {
            res+= "(int)((AbstractProperty*)(";
            s =  m_parent_list.back ()->get_symbol (s.substr (1, s.length() - 2));
            res+=s;
            if (is_alone) {
              res+="))->get_double_value())";
              if (i != sym.second.size ()-1) {
                res += "->find_child(\"";
              }
            } else {
              res+="->find_child(\"";
            }
          }

        } else if (s[s.length()-1] == ']') {
          res+=s.substr (0, s.length() - 2);
          res+="\"))->get_double_value())";
          if (i != sym.second.size ()-1) {
            res += "->find_child(\"";
          }
        } else {
          if (i == 0)
            res +="\"";
          res += s;
          if ( (i != sym.second.size ()-1) && sym.second.at(i+1)[0] != '[')
            res += "/";
        }
      }
      res+= "\")";
    }
    return  res;
  }

  std::string
  CPPBuilder::build_root_and_path (const std::pair< std::string, std::vector<std::string> > &sym)
  {
    for (auto s: sym.second) {
      if (s[0] == '[')
        return std::string (build_find_child(nullptr, sym) + ", \"\"");
    }
    std::string s = sym.first + ", \"";
    if (!sym.second.empty()){
      std::string last = sym.second.back ();
      for (int i = 0; i < sym.second.size (); i++) {
        auto ns = sym.second.at(i);
        s += ns;
        if (i != sym.second.size ()-1)
          s+= "/";
      }
    }
    s += "\"";
    return s;
  }

  void
  CPPBuilder::print_args (std::ofstream &os,
                          std::vector<std::pair<ParamType, std::string> > args,
                          bool is_first)
  {
    int size = args.size ();
    if (size == 0)
      return;
    if (!is_first)
      os << ", ";
    for (int i = 0; i < size; i++) {
      std::string str;
      if (args.at (i).first == NAME) {
        std::pair<std::string, std::vector<std::string>> var = parse_symbol (
            args.at (i).second);
        str =
            var.second.empty () ?
                var.first : build_find_child (nullptr, var);
      } else {
        str = args.at (i).second;
      }
      os << str;
      if (i < size - 1)
        os << ", ";
    }
  }
} /* namespace Smala */
