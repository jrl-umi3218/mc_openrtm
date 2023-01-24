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
#include <mc_rtc/version.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include "hrpsys/io/iob.h"

using steady_clock = std::chrono::steady_clock;

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
    "conf.default.is_simulation", "0",
    ""
  };
// </rtc-template>

namespace
{
  bool init_controller()
  {
    if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
    {
      mc_rtc::log::error("MCControl was compiled with {}  but mc_rtc is at version {}, you might face subtle issues or unexpected crashes, please recompile mc_openrtm", mc_rtc::MC_RTC_VERSION, mc_rtc::version());
    }
    return false;
  }
}

MCControl::MCControl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_timeStep(0.002),
    m_is_simulation(false),
    m_qInIn("qIn", m_qIn),
    m_alphaInIn("alphaIn", m_alphaIn),
    m_qInitIn("qInit", m_qInit),
    m_pInIn("pIn", m_pIn),
    m_rpyInIn("rpyIn", m_rpyIn),
    m_rateInIn("rateIn", m_rateIn),
    m_accInIn("accIn", m_accIn),
    m_poseInIn("poseIn", m_poseIn),
    m_velInIn("velIn", m_velIn),
    m_taucInIn("taucIn", m_taucIn),
    m_motorTempNames(),
    m_motorTempToRJOIndex(),
    m_motorTempInIn("motorTempIn", m_motorTempIn),
    m_basePoseInIn("basePoseIn", m_basePoseIn),
    m_baseVelInIn("baseVelIn", m_baseVelIn),
    m_baseAccInIn("baseAccIn", m_baseAccIn),
    m_pgainsInIn("pgainsIn", m_pgainsIn),
    m_dgainsInIn("dgainsIn", m_dgainsIn),
    m_wrenchesNames(),
    m_qOutOut("qOut", m_qOut),
    m_pOutOut("pOut", m_pOut),
    m_rpyOutOut("rpyOut", m_rpyOut),
    m_pgainsOutOut("pgainsOut", m_pgainsOut),
    m_dgainsOutOut("dgainsOut", m_dgainsOut),
    m_MCControlServicePortPort("MCControlServicePort"),
    m_service0(this),
    init(init_controller())
    // </rtc-template>
// clang-format on
{
  const auto & rm = controller.robot().module();
  for(const auto & fs : rm.forceSensors())
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

  const auto & rjo = controller.robot().refJointOrder();
  for(const auto & js : rm.jointSensors())
  {
    m_motorTempNames.push_back(js.joint());
    auto rjo_it = std::find(rjo.begin(), rjo.end(), js.joint());
    if(rjo_it != rjo.end())
    {
      int rjo_idx = std::distance(rjo.begin(), rjo_it);
      m_motorTempToRJOIndex.push_back(rjo_idx);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "RobotModule contains a JointSensor at {} but it was not found in the refJointOrder.", js.joint());
    }
  }

  // create datastore calls for reading/writing servo pd gains
  controller.controller().datastore().make_call(
      controller.robot().name() + "::GetPDGains",
      [this](std::vector<double> & p, std::vector<double> & d) { return getServoGains(p, d); });
  controller.controller().datastore().make_call(
      controller.robot().name() + "::GetPDGainsByName",
      [this](const std::string & jn, double & p, double & d) { return getServoGainsByName(jn, p, d); });
  controller.controller().datastore().make_call(
      controller.robot().name() + "::SetPDGains",
      [this](const std::vector<double> & p, const std::vector<double> & d) { return setServoGains(p, d); });
  controller.controller().datastore().make_call(
      controller.robot().name() + "::SetPDGainsByName",
      [this](const std::string & jn, double p, double d) { return setServoGainsByName(jn, p, d); });
}

MCControl::~MCControl() {}

bool MCControl::getServoGains(std::vector<double> & p_vec, std::vector<double> & d_vec)
{
  if(!m_is_simulation)
  {
    if(!open_iob())
    {
      return failed_iob("open_iob", __func__);
    }
    size_t num_joints = number_of_joints();
    p_vec.resize(num_joints);
    d_vec.resize(num_joints);

    for(unsigned int i = 0; i < num_joints; i++)
    {
      if(!read_pgain(i, &p_vec[i]))
      {
        return failed_iob("read_pgain for joint " + i, __func__);
      };
      if(!read_dgain(i, &d_vec[i]))
      {
        return failed_iob("read_dgain for joint " + i, __func__);
      };
    }
    if(!close_iob())
    {
      return failed_iob("close_iob", __func__);
    }
  }
  else
  {
    // if in simulation
    m_pgainsInIn.read();
    m_dgainsInIn.read();
    size_t num_joints = m_pgainsIn.data.length();
    p_vec.resize(num_joints);
    d_vec.resize(num_joints);
    std::memcpy(&p_vec[0], &m_pgainsIn.data[0], num_joints * sizeof(double));
    std::memcpy(&d_vec[0], &m_dgainsIn.data[0], num_joints * sizeof(double));
  }
  return true;
}

bool MCControl::getServoGainsByName(const std::string & jn, double & p, double & d)
{
  const auto & rjo = controller.robot().refJointOrder();
  auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
  if(rjo_it == rjo.end())
  {
    mc_rtc::log::warning("[mc_openrtm] {}::SetPDGainsByName failed. Joint {} not found in ref_joint_order.",
                         controller.robot().name(), jn);
    return false;
  }
  int rjo_idx = std::distance(rjo.begin(), rjo_it);
  if(!m_is_simulation)
  {
    // if on real robot
    if(!open_iob())
    {
      return failed_iob("open_iob", __func__);
    };
    if(!read_pgain(rjo_idx, &p))
    {
      return failed_iob("read_pgain for joint " + rjo_idx, "getServoGainsByName");
    };
    if(!read_dgain(rjo_idx, &d))
    {
      return failed_iob("read_dgain for joint " + rjo_idx, "getServoGainsByName");
    };
    if(!close_iob())
    {
      return failed_iob("close_iob", __func__);
    };
  }
  else
  {
    // if in simulation
    m_pgainsInIn.read();
    m_dgainsInIn.read();
    p = m_pgainsIn.data[rjo_idx];
    d = m_dgainsIn.data[rjo_idx];
  }
  return true;
}

bool MCControl::setServoGains(const std::vector<double> & p_vec, const std::vector<double> & d_vec)
{
  const auto & rjo = controller.robot().refJointOrder();
  if(p_vec.size() != rjo.size())
  {
    return false;
  }
  if(d_vec.size() != rjo.size())
  {
    return false;
  }

  if(!m_is_simulation)
  {
    // if on real robot
    if(!open_iob())
    {
      return failed_iob("open_iob", __func__);
    };
    for(unsigned int i = 0; i < rjo.size(); i++)
    {
      if(!write_pgain(i, p_vec[i]))
      {
        return failed_iob("write_pgain for joint " + i, __func__);
      };
      if(!write_dgain(i, d_vec[i]))
      {
        return failed_iob("write_dgain for joint " + i, __func__);
      };
    }
    if(!close_iob())
    {
      return failed_iob("close_iob", __func__);
    };
    ;
  }
  else
  {
    // if in simulation
    m_pgainsOut.data.length(rjo.size());
    m_dgainsOut.data.length(rjo.size());
    for(unsigned int i = 0; i < rjo.size(); i++)
    {
      m_pgainsOut.data[i] = p_vec[i];
      m_dgainsOut.data[i] = d_vec[i];
    }

    coil::TimeValue coiltm(coil::gettimeofday());
    RTC::Time tm;
    tm.sec = static_cast<CORBA::ULong>(coiltm.sec());
    tm.nsec = static_cast<CORBA::ULong>(coiltm.usec()) * 1000;
    m_pgainsOut.tm = tm;
    m_dgainsOut.tm = tm;
    m_pgainsOutOut.write();
    m_dgainsOutOut.write();
  }
  return true;
}

bool MCControl::setServoGainsByName(const std::string & jn, double p, double d)
{
  const auto & rjo = controller.robot().refJointOrder();
  auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
  if(rjo_it == rjo.end())
  {
    mc_rtc::log::warning("[mc_openrtm] {}::SetPDGainsByName failed. Joint {} not found in ref_joint_order.",
                         controller.robot().name(), jn);
    return false;
  }
  int rjo_idx = std::distance(rjo.begin(), rjo_it);
  if(!m_is_simulation)
  {
    // if on real robot
    if(!open_iob())
    {
      return failed_iob("open_iob", __func__);
    };
    if(!write_pgain(rjo_idx, p))
    {
      return failed_iob("write_pgain for joint " + rjo_idx, "setServoGainsByName");
    };
    if(!write_dgain(rjo_idx, d))
    {
      return failed_iob("write_dgain for joint " + rjo_idx, "setServoGainsByName");
    };
    if(!close_iob())
    {
      return failed_iob("close_iob", __func__);
    };
    ;
  }
  else
  {
    // if in simulation
    std::vector<double> p_vec;
    std::vector<double> d_vec;
    getServoGains(p_vec, d_vec);
    p_vec[rjo_idx] = p;
    d_vec[rjo_idx] = d;
    m_pgainsOut.data.length(rjo.size());
    m_dgainsOut.data.length(rjo.size());
    for(unsigned int i = 0; i < rjo.size(); i++)
    {
      m_pgainsOut.data[i] = p_vec[i];
      m_dgainsOut.data[i] = d_vec[i];
    }

    coil::TimeValue coiltm(coil::gettimeofday());
    RTC::Time tm;
    tm.sec = static_cast<CORBA::ULong>(coiltm.sec());
    tm.nsec = static_cast<CORBA::ULong>(coiltm.usec()) * 1000;
    m_pgainsOut.tm = tm;
    m_dgainsOut.tm = tm;
    m_pgainsOutOut.write();
    m_dgainsOutOut.write();
  }
  return true;
}

RTC::ReturnCode_t MCControl::onInitialize()
{
  mc_rtc::log::info("MCControl::onInitialize() starting");
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qIn", m_qInIn);
  addInPort("alphaIn", m_alphaInIn);
  addInPort("qInit", m_qInitIn);
  addInPort("pIn", m_pInIn);
  addInPort("rpyIn", m_rpyInIn);
  addInPort("rateIn", m_rateInIn);
  addInPort("accIn", m_accInIn);
  addInPort("poseIn", m_poseInIn);
  addInPort("velIn", m_velInIn);
  addInPort("taucIn", m_taucInIn);
  addInPort("pgainsIn", m_pgainsInIn);
  addInPort("dgainsIn", m_dgainsInIn);
  addInPort("motorTempIn", m_motorTempInIn);
  // Floating base
  addInPort("basePoseIn", m_basePoseInIn);
  addInPort("baseVelIn", m_baseVelInIn);
  addInPort("baseAccIn", m_baseAccInIn);

  // Force sensors
  for(size_t i = 0; i < m_wrenchesNames.size(); ++i)
  {
    addInPort(m_wrenchesNames[i].c_str(), *(m_wrenchesInIn[i]));
  }

  // Set OutPort buffer
  addOutPort("qOut", m_qOutOut);
  addOutPort("pOut", m_pOutOut);
  addOutPort("rpyOut", m_rpyOutOut);
  addOutPort("pgainsOut", m_pgainsOutOut);
  addOutPort("dgainsOut", m_dgainsOutOut);

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
  bindParameter("is_simulation", m_is_simulation, "0");

  // </rtc-template>
  mc_rtc::log::info("MCControl::onInitialize() finished");
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCControl::onActivated(RTC::UniqueId ec_id)
{
  prev_start_t = steady_clock::now();
  mc_rtc::log::info("onActivated");
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCControl::onDeactivated(RTC::UniqueId ec_id)
{
  mc_rtc::log::info("onDeactivated");
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCControl::onExecute(RTC::UniqueId ec_id)
{
  auto start_t = steady_clock::now();
  loop_dt = start_t - prev_start_t;
  double solver_dt = controller.controller().solver().dt();
  if(!m_is_simulation && loop_dt.count() > 1.5 * 1000 * solver_dt)
  {
    mc_rtc::log::warning("[mc_openrtm] LoopDt exceeds expected time: {}ms (expected: {}ms)", loop_dt.count(),
                         1000 * solver_dt);
  }
  prev_start_t = start_t;

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
  if(m_basePoseInIn.isNew())
  {
    m_basePoseInIn.read();
    floatingBasePosIn.x() = m_basePoseIn.data.position.x;
    floatingBasePosIn.y() = m_basePoseIn.data.position.y;
    floatingBasePosIn.z() = m_basePoseIn.data.position.z;
    floatingBaseRPYIn.x() = m_basePoseIn.data.orientation.r;
    floatingBaseRPYIn.y() = m_basePoseIn.data.orientation.p;
    floatingBaseRPYIn.z() = m_basePoseIn.data.orientation.y;
  }
  if(m_baseVelInIn.isNew())
  {
    m_baseVelInIn.read();
    if(m_baseVelIn.data.length() == 6)
    {
      floatingBaseVelIn.angular().x() = m_baseVelIn.data[3];
      floatingBaseVelIn.angular().y() = m_baseVelIn.data[4];
      floatingBaseVelIn.angular().z() = m_baseVelIn.data[5];
      floatingBaseVelIn.linear().x() = m_baseVelIn.data[0];
      floatingBaseVelIn.linear().y() = m_baseVelIn.data[1];
      floatingBaseVelIn.linear().z() = m_baseVelIn.data[2];
    }
  }
  if(m_baseAccInIn.isNew())
  {
    m_baseAccInIn.read();
    if(m_baseAccIn.data.length() == 6)
    {
      floatingBaseAccIn.angular().x() = m_baseAccIn.data[3];
      floatingBaseAccIn.angular().y() = m_baseAccIn.data[4];
      floatingBaseAccIn.angular().z() = m_baseAccIn.data[5];
      floatingBaseAccIn.linear().x() = m_baseAccIn.data[0];
      floatingBaseAccIn.linear().y() = m_baseAccIn.data[1];
      floatingBaseAccIn.linear().z() = m_baseAccIn.data[2];
    }
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
  if(m_alphaInIn.isNew())
  {
    m_alphaInIn.read();
    alphaIn.resize(m_alphaIn.data.length());
    for(unsigned int i = 0; i < alphaIn.size(); ++i)
    {
      alphaIn[i] = m_alphaIn.data[i];
    }
  }
  if(m_motorTempInIn.isNew())
  {
    m_motorTempInIn.read();
    for(unsigned int i = 0; i < m_motorTempNames.size(); i++)
    {
      motorTempIn[m_motorTempNames[i]] = m_motorTempIn.data[m_motorTempToRJOIndex[i]];
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
    controller.setSensorLinearAcceleration(accIn);
    controller.setEncoderValues(qIn);
    controller.setEncoderVelocities(alphaIn);
    controller.setWrenches(m_wrenches);
    controller.setJointTorques(taucIn);
    controller.setJointMotorTemperatures(motorTempIn);

    // Floating base sensor
    if(controller.robot().hasBodySensor("FloatingBase"))
    {
      controller.setSensorPositions({{"FloatingBase", floatingBasePosIn}});
      controller.setSensorOrientations({{"FloatingBase", Eigen::Quaterniond(mc_rbdyn::rpyToMat(floatingBaseRPYIn))}});
      controller.setSensorAngularVelocities({{"FloatingBase", floatingBaseVelIn.angular()}});
      controller.setSensorLinearVelocities({{"FloatingBase", floatingBaseVelIn.linear()}});
      controller.setSensorAngularAccelerations({{"FloatingBase", floatingBaseAccIn.angular()}});
      controller.setSensorLinearAccelerations({{"FloatingBase", floatingBaseAccIn.linear()}});
    }

    if(controller.running)
    {
      if(!init)
      {
        mc_rtc::log::info("Init controller");
        if(qInit.size() == qIn.size())
        {
          controller.init(qInit);
        }
        else
        {
          mc_rtc::log::warning("The size of qInit does not mach the size of qIn, please update the corresponding "
                               "Python script... using qIn instead to initialize the controller");
          controller.init(qIn);
        }
        controller.controller().logger().addLogEntry("perf_LoopDt", [&]() { return loop_dt.count(); });
        init = true;
      }
      if(controller.run())
      {
        m_qOut.data.length(m_qIn.data.length());
        const auto & ref_joint_order = controller.robot().refJointOrder();
        const auto & qOut = controller.robot().mbc().q;
        for(size_t i = 0; i < ref_joint_order.size(); ++i)
        {
          auto jIdx = controller.robot().jointIndexInMBC(i);
          if(jIdx != -1 && qOut[static_cast<size_t>(jIdx)].size() == 1)
          {
            m_qOut.data[i] = qOut[static_cast<size_t>(jIdx)][0];
          }
          else
          {
            m_qOut.data[i] = m_qIn.data[i];
          }
        }
        /* FIXME Correction RPY convention here? */
        const auto & ff_state = qOut[0];
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
