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

#include "node.h"
#include "term_node.h"
namespace Smala
{

  class TransitionNode : public Node
  {
  public:
    TransitionNode ();
    TransitionNode (const std::string &type, const std::string &name, const std::string &src_name, const std::string &dst_name, PathNode* trigger, PathNode* action);
    virtual ~TransitionNode () {}

    std::string& get_src () { return m_src; }
    void set_src (const std::string& src) { m_src = src; }
    std::string& get_dst () { return m_dst; }
    void set_dst (std::string &dst) { m_dst = dst; }
    PathNode* get_trigger () { return m_trigger; }
    PathNode* get_action () { return m_action; }

  private:
    PathNode *m_trigger;
    PathNode *m_action;
    std::string m_src, m_dst;
  };

} /* namespace Smala */
