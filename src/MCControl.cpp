/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// -*- C++ -*-
/*!
 * @file  MCControl.cpp * @brief Core component for MC control * $Date$
 *
 * $Id$
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#  pragma GCC diagnostic ignored "-Wdelete-incomplete"
#  pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#include "MCControl.h"

#include <fstream>
#include <iomanip>

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

// Module specification
// clang-format off
// <rtc-template block="module_spec">
static const char* mccontrol_spec[] =
  {
    "implementation_id", "MCControl",
    "type_name",         "MCControl",
    "description",       "Core component for MC control",
    "version",           "0.1",
    "vendor",            "CNRS",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.timeStep", "0.002",
    "conf.default.is_enabled", "0",
    ""
  };
// </rtc-template>

MCControl::MCControl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_timeStep(0.002),
    m_enabled(false),
    m_qInIn("qIn", m_qIn),
    m_qInitIn("qInit", m_qInit),
    m_pInIn("pIn", m_pIn),
    m_rpyInIn("rpyIn", m_rpyIn),
    m_rateInIn("rateIn", m_rateIn),
    m_accInIn("accIn", m_accIn),
    m_poseInIn("poseIn", m_poseIn),
    m_velInIn("velIn", m_velIn),
    m_taucInIn("taucIn", m_taucIn),
    m_wrenchesNames(),
    m_qOutOut("qOut", m_qOut),
    m_pOutOut("pOut", m_pOut),
    m_rpyOutOut("rpyOut", m_rpyOut),
    m_MCControlServicePortPort("MCControlServicePort"),
    m_service0(this),
    init(false)
    // </rtc-template>
// clang-format on
{
  auto rm = controller.get_robot_module();
  for(const auto & fs : rm->forceSensors())
  {
    m_wrenchesNames.push_back(fs.name());
  }
  m_wrenchesIn.resize(0);
  m_wrenchesInIn.resize(0);
  for(size_t i = 0; i < m_wrenchesNames.size(); ++i)
  {
    const auto & wrenchName = m_wrenchesNames[i];
    m_wrenchesIn.push_back(new TimedDoubleSeq());
    m_wrenchesInIn.push_back(new InPort<TimedDoubleSeq>(wrenchName.c_str(), *(m_wrenchesIn[i])));
    m_wrenches[wrenchName] = sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
  }
}

MCControl::~MCControl() {}

RTC::ReturnCode_t MCControl::onInitialize()
{
  LOG_INFO("MCControl::onInitialize() starting")
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qIn", m_qInIn);
  addInPort("qInit", m_qInitIn);
  addInPort("pIn", m_pInIn);
  addInPort("rpyIn", m_rpyInIn);
  addInPort("rateIn", m_rateInIn);
  addInPort("accIn", m_accInIn);
  addInPort("poseIn", m_poseInIn);
  addInPort("velIn", m_velInIn);
  addInPort("taucIn", m_taucInIn);
  for(size_t i = 0; i < m_wrenchesNames.size(); ++i)
  {
    addInPort(m_wrenchesNames[i].c_str(), *(m_wrenchesInIn[i]));
  }

  // Set OutPort buffer
  addOutPort("qOut", m_qOutOut);
  addOutPort("pOut", m_pOutOut);
  addOutPort("rpyOut", m_rpyOutOut);

  // Set service provider to Ports
  m_MCControlServicePortPort.registerProvider("service0", "MCControlService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_MCControlServicePortPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("timeStep", m_timeStep, "0.002");
  bindParameter("is_enabled", controller.running, "0");

  auto gripperJs = controller.gripperJoints();
  auto gripperActiveJs = controller.gripperActiveJoints();
  const auto & ref_joint_order = controller.ref_joint_order();
  for(const auto & g : gripperActiveJs)
  {
    gripper_in_index[g.first] = {};
    realGripperQs[g.first] = {};
    for(const auto & jn : g.second)
    {
      for(size_t i = 0; i < ref_joint_order.size(); ++i)
      {
        if(ref_joint_order[i] == jn)
        {
          gripper_in_index[g.first].push_back(i);
          realGripperQs[g.first].push_back(0.0);
        }
      }
    }
  }
  for(const auto & g : gripperJs)
  {
    gripper_out_index[g.first] = {};
    for(size_t j = 0; j < g.second.size(); ++j)
    {
      const auto & jn = g.second[j];
      for(size_t i = 0; i < ref_joint_order.size(); ++i)
      {
        if(ref_joint_order[i] == jn)
        {
          gripper_out_index[g.first].push_back({i, j});
        }
      }
    }
  }

  // </rtc-template>
  LOG_INFO("MCControl::onInitialize() finished")
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCControl::onActivated(RTC::UniqueId ec_id)
{
  LOG_INFO("onActivated")
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCControl::onDeactivated(RTC::UniqueId ec_id)
{
  LOG_INFO("onDeactivated")
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCControl::onExecute(RTC::UniqueId ec_id)
{
  for(size_t i = 0; i < m_wrenchesInIn.size(); ++i)
  {
    if(m_wrenchesInIn[i]->isNew())
    {
      m_wrenchesInIn[i]->read();
      if(m_wrenchesIn[i]->data.length() == 6)
      {
        m_wrenches[m_wrenchesNames[i]].force() =
            Eigen::Vector3d(m_wrenchesIn[i]->data[0], m_wrenchesIn[i]->data[1], m_wrenchesIn[i]->data[2]);
        m_wrenches[m_wrenchesNames[i]].couple() =
            Eigen::Vector3d(m_wrenchesIn[i]->data[3], m_wrenchesIn[i]->data[4], m_wrenchesIn[i]->data[5]);
      }
    }
  }
  if(m_poseInIn.isNew())
  {
    m_poseInIn.read();
    m_pIn.data = m_poseIn.data.position;
    m_rpyIn.data = m_poseIn.data.orientation;
    rpyIn(0) = m_rpyIn.data.r;
    rpyIn(1) = m_rpyIn.data.p;
    rpyIn(2) = m_rpyIn.data.y;
    pIn(0) = m_pIn.data.x;
    pIn(1) = m_pIn.data.y;
    pIn(2) = m_pIn.data.z;
  }
  if(m_velInIn.isNew())
  {
    m_velInIn.read();
    velIn.linear()(0) = m_velIn.data[0];
    velIn.linear()(1) = m_velIn.data[1];
    velIn.linear()(2) = m_velIn.data[2];
    velIn.angular()(0) = m_velIn.data[3];
    velIn.angular()(1) = m_velIn.data[4];
    velIn.angular()(2) = m_velIn.data[5];
  }
  if(m_pInIn.isNew())
  {
    m_pInIn.read();
    pIn(0) = m_pIn.data.x;
    pIn(1) = m_pIn.data.y;
    pIn(2) = m_pIn.data.z;
  }
  if(m_rpyInIn.isNew())
  {
    m_rpyInIn.read();
    rpyIn(0) = m_rpyIn.data.r;
    rpyIn(1) = m_rpyIn.data.p;
    rpyIn(2) = m_rpyIn.data.y;
  }
  if(m_rateInIn.isNew())
  {
    m_rateInIn.read();
    rateIn(0) = m_rateIn.data.avx;
    rateIn(1) = m_rateIn.data.avy;
    rateIn(2) = m_rateIn.data.avz;
  }
  if(m_accInIn.isNew())
  {
    m_accInIn.read();
    accIn(0) = m_accIn.data.ax;
    accIn(1) = m_accIn.data.ay;
    accIn(2) = m_accIn.data.az;
  }
  if(m_taucInIn.isNew())
  {
    m_taucInIn.read();
    taucIn.resize(m_taucIn.data.length());
    for(unsigned int i = 0; i < static_cast<unsigned int>(m_taucIn.data.length()); ++i)
    {
      taucIn[i] = m_taucIn.data[i];
    }
  }
  if(m_qInitIn.isNew())
  {
    m_qInitIn.read();
    qInit.resize(m_qInit.data.length());
    for(unsigned int i = 0; i < qInit.size(); ++i)
    {
      qInit[i] = m_qInit.data[i];
    }
  }
  if(m_qInIn.isNew())
  {
    m_qInIn.read();
    qIn.resize(m_qIn.data.length());
    for(unsigned int i = 0; i < qIn.size(); ++i)
    {
      qIn[i] = m_qIn.data[i];
    }
    coil::TimeValue coiltm(coil::gettimeofday());
    RTC::Time tm;
    tm.sec = static_cast<CORBA::ULong>(coiltm.sec());
    tm.nsec = static_cast<CORBA::ULong>(coiltm.usec()) * 1000;
    controller.setSensorOrientation(Eigen::Quaterniond(mc_rbdyn::rpyToMat(rpyIn)).normalized());
    controller.setSensorPosition(pIn);
    controller.setSensorAngularVelocity(rateIn);
    controller.setSensorLinearVelocity(velIn.linear());
    controller.setSensorAcceleration(accIn);
    controller.setEncoderValues(qIn);
    controller.setWrenches(m_wrenches);
    controller.setJointTorques(taucIn);
    if(controller.running)
    {
      double t = tm.sec * 1e9 + tm.nsec;
      if(!init)
      {
        LOG_INFO("Init controller")
        auto q = Eigen::Quaterniond(mc_rbdyn::rpyToMat(rpyIn)).normalized();
        controller.init(qIn, std::array<double, 7>{{q.w(), q.x(), q.y(), q.z(), pIn.x(), pIn.y(), pIn.z()}});
        init = true;
      }
      if(controller.run())
      {
        const mc_solver::QPResultMsg & res = controller.send(t);
        auto gripperQs = controller.gripperQ();
        const auto & ref_joint_order = controller.ref_joint_order();
        for(auto & rG : realGripperQs)
        {
          const auto & idx = gripper_in_index[rG.first];
          auto & qs = rG.second;
          for(size_t i = 0; i < idx.size(); ++i)
          {
            qs[i] = m_qIn.data[idx[i]];
          }
        }
        controller.setActualGripperQ(realGripperQs);
        m_qOut.data.length(m_qIn.data.length());
        for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
        {
          if(res.robots_state[0].q.count(ref_joint_order[i]))
          {
            m_qOut.data[i] = res.robots_state[0].q.at(ref_joint_order[i])[0];
          }
          else
          {
            m_qOut.data[i] = m_qIn.data[i];
          }
        }
        /* Update gripper state */
        for(const auto & cG : gripper_out_index)
        {
          const auto & qs = gripperQs[cG.first];
          for(const auto & idx_p : cG.second)
          {
            m_qOut.data[idx_p.first] = qs[idx_p.second];
          }
        }
        /* FIXME Correction RPY convention here? */
        const auto & ff_state = res.robots_state[0].q.at(controller.robot().mb().joint(0).name());
        if(ff_state.size())
        {
          Eigen::Vector3d rpyOut = Eigen::Quaterniond(ff_state[0], ff_state[1], ff_state[2], ff_state[3])
                                       .toRotationMatrix()
                                       .eulerAngles(2, 1, 0);
          m_rpyOut.data.r = rpyOut[2];
          m_rpyOut.data.p = rpyOut[1];
          m_rpyOut.data.y = rpyOut[0];

          m_pOut.data.x = ff_state[4];
          m_pOut.data.y = ff_state[5];
          m_pOut.data.z = ff_state[6];
        }
      }

      m_qOut.tm = tm;
      m_rpyOut.tm = tm;
      m_pOut.tm = tm;
      m_qOutOut.write();
      m_pOutOut.write();
      m_rpyOutOut.write();
    }
    else
    {
      init = false;
      m_qOut = m_qIn;
      /* Still run controller.run() in order to handle some service calls */
      mc_rbdyn::Robot & robot = const_cast<mc_rbdyn::Robot &>(controller.robot());
      std::vector<std::vector<double>> q = robot.mbc().q;
      if(q[0].size() == 7)
      {
        q[0] = {1, 0, 0, 0, 0, 0, 0.76};
      }
      const std::vector<double> & initq = qIn;
      const auto & ref_joint_order = controller.ref_joint_order();
      for(size_t i = 0; i < ref_joint_order.size(); ++i)
      {
        const auto & jN = ref_joint_order[i];
        if(robot.hasJoint(jN))
        {
          q[robot.jointIndexByName(jN)] = {initq[i]};
        }
      }
      if(qInit.size() > 0)
      {
        auto gripperQs = controller.gripperQ();
        auto gripperJs = controller.gripperActiveJoints();
        std::map<std::string, std::vector<double>> realGripperQs;
        for(const auto & g : gripperJs)
        {
          realGripperQs[g.first] = {};
          for(const auto & jn : g.second)
          {
            for(size_t i = 0; i < ref_joint_order.size(); ++i)
            {
              if(ref_joint_order[i] == jn && qInit.size() > i)
              {
                realGripperQs[g.first].push_back(qInit[i]);
                break;
              }
            }
          }
        }
        controller.setGripperCurrentQ(realGripperQs);
        for(const auto & gn : realGripperQs)
        {
          controller.setGripperTargetQ(gn.first, gn.second);
        }
      }
      robot.mbc().q = q;
      rbd::forwardKinematics(robot.mb(), robot.mbc());
      rbd::forwardVelocity(robot.mb(), robot.mbc());
      controller.run();
    }
  }
  return RTC::RTC_OK;
}

extern "C"
{

  void MCControlInit(RTC::Manager * manager)
  {
    coil::Properties profile(mccontrol_spec);
    manager->registerFactory(profile, RTC::Create<MCControl>, RTC::Delete<MCControl>);
  }
};

#pragma GCC diagnostic pop
