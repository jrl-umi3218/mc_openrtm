/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "MCControlServiceSVC_impl.h"

/* Trick to disable compiler warning on this part of the code */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#  pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#pragma GCC diagnostic pop

#include <iostream>

#include "MCControl.h"

namespace OpenHRP
{

MCControlServiceSVC_impl::MCControlServiceSVC_impl(MCControl * plugin) : m_plugin(plugin) {}

MCControlServiceSVC_impl::~MCControlServiceSVC_impl() {}

CORBA::Boolean MCControlServiceSVC_impl::EnableController(const char * name)
{
  return m_plugin->controller.EnableController(name);
}

CORBA::Boolean MCControlServiceSVC_impl::open_grippers()
{
  m_plugin->controller.setGripperOpenPercent(1);
  return true;
}

CORBA::Boolean MCControlServiceSVC_impl::close_grippers()
{
  m_plugin->controller.setGripperOpenPercent(0);
  return true;
}

CORBA::Boolean MCControlServiceSVC_impl::set_gripper(const char * gripper, const OpenHRP::DblSequence & v)
{
  std::vector<double> q;
  for(unsigned int i = 0; i < v.length(); ++i)
  {
    q.push_back(v[i]);
  }
  m_plugin->controller.setGripperTargetQ(gripper, q);
  return true;
}

} // namespace OpenHRP
