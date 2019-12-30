/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_MCCONTROLSERVICESVCIMPL_H_
#define _H_MCCONTROLSERVICESVCIMPL_H_

#include "MCControlService.hh"

class MCControl;

namespace OpenHRP
{
  class MCControlServiceSVC_impl:
    public virtual POA_OpenHRP::MCControlService,
    public virtual PortableServer::RefCountServantBase
  {
    public:
      MCControlServiceSVC_impl(MCControl * plugin);
      virtual ~MCControlServiceSVC_impl();

      virtual CORBA::Boolean EnableController(const char * name);

      /* Grippers (always available) */
      virtual CORBA::Boolean open_grippers() override;

      virtual CORBA::Boolean close_grippers() override;

      virtual CORBA::Boolean set_gripper(const char * gripper, const OpenHRP::DblSequence & v) override;

    private:
      MCControl * m_plugin;
  };
}

#endif
