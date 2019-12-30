#!/usr/bin/python

import rtm
import socket
# import robot
import sys

# from rtm import *
from rtm import connectPorts
from rtm import narrow
from rtm import isJython

# import OpenHRP
from OpenHRP import *
if isJython():
    from OpenHRP.RobotHardwareServicePackage import *

rtm.nsport = 2809
rtm.nshost = "localhost"

try:
    import cnoid.Corba
    from cnoid.grxui import *
    orb = cnoid.Corba.getORB()
    rtm.orb = orb
    if rtm.nshost is None:
        rtm.nshost = socket.gethostname()
    nsloc = "corbaloc:iiop:%s:%d/NameService" % (rtm.nshost, rtm.nsport)
    print(nsloc)
    rtm.initCORBA()
    sys.argv = []
    inChoreonoid = True
except Exception:
    rtm.initCORBA()
    inChoreonoid = False

def connectComps():
    connectPorts(rh.port("q"), [sh.port("currentQIn")])
    connectPorts(rh.port("gyrometer"), [kf.port("rate")])
    connectPorts(rh.port("gsensor"), [kf.port("acc")])
    # for the kinematics mode
    if kinematics_mode == 1:
        connectPorts(sh.port("qOut"), rh.port("qIn"))
        connectPorts(sh.port("baseTformOut"), rh.port("baseTformIn"))
    #
    if not rh.port("baseTform"):
        # connectPorts(st.port("qRefOut"), servo.port("qRef"))
        connectPorts(sh.port("qOut"), servo.port("angleRef"))
    else:
        # connectPorts(st.port("qRefOut"), rh.port("qRef"))
        connectPorts(sh.port("baseTformOut"), rh.port("baseTform"))


def createComps():
    global ms, rh, rh_svc, sh, sh_svc, tk_svc, st, kf, log, log_svc, servo
    global ep_svc, mc, mc_svc

    rh = rtm.findRTC("RobotHardware0")
    if rh is not None:
        rh_svc = narrow(rh.service("service0"), "RobotHardwareService")
        servo = rh
        ep_svc = narrow(rh.ec, "ExecutionProfileService")
    else:
        rh = rtm.findRTC("JVRC1Controller(Robot)0")
        servo = rtm.findRTC("PDcontroller0")

    ms = rtm.findRTCmanager()

    ms.load("KalmanFilter")
    kf = ms.create("KalmanFilter")

    ms.load("StateHolder")
    sh = ms.create("StateHolder")
    sh_svc = narrow(sh.service("service0"), "StateHolderService")
    tk_svc = narrow(sh.service("service1"), "TimeKeeperService")

    ms.load("DataLogger")
    log = ms.create("DataLogger")
    log_svc = narrow(log.service("service0"), "DataLoggerService")

    ms.load("MCControl")
    mc = ms.create("MCControl")

def activateComps():
    rtm.serializeComponents([rh, kf, log, mc, sh])
    rh.start()
    kf.start()
    sh.start()
    log.start()
    mc.start()

def init(hostname=socket.gethostname()):
    global ms, simulation_mode, kinematics_mode

    ms = rtm.findRTCmanager()

    rh = rtm.findRTC("RobotHardware0")
    if rh is not None:
        rh_svc = narrow(rh.service("service0"), "RobotHardwareService")
        servo = rh
        ep_svc = narrow(rh.ec, "ExecutionProfileService")
        simulation_mode = 0
    else:
        rh = rtm.findRTC("JVRC1Controller(Robot)0")
        servo = rtm.findRTC("PDcontroller0")
        simulation_mode = 1
        if rh.port("baseTformIn"):
            kinematics_mode = 1
        else:
            kinematics_mode = 0

    print("creating components")
    createComps()

    print("connecting components")
    connectComps()

    print("activating components")
    activateComps()

    print("initialized successfully")


def startMCControl():
    global mc
    mc.setProperty("is_enabled", "1")

def connectMCControl():
    connectPorts(rh.port("q"), mc.port("qIn"))
    connectPorts(rh.port("gyrometer"), mc.port("rateIn"))
    connectPorts(rh.port("gsensor"), mc.port("accIn"))
    connectPorts(rh.port("rfsensor"), mc.port("RightFootForceSensor"))
    connectPorts(rh.port("lfsensor"), mc.port("LeftFootForceSensor"))
    connectPorts(rh.port("rhsensor"), mc.port("RightHandForceSensor"))
    connectPorts(rh.port("lhsensor"), mc.port("LeftHandForceSensor"))
    connectPorts(sh.port("basePosOut"), mc.port("pIn"))
    connectPorts(kf.port("rpy"), mc.port("rpyIn"))
    connectPorts(rh.port("waistAbsTransform"), mc.port("basePoseIn"))
    connectPorts(rh.port("waistAbsVelocity"), mc.port("baseVelIn"))
    connectPorts(rh.port("waistAbsAcceleration"), mc.port("baseAccIn"))
    connectPorts(mc.port("qOut"), sh.port("qIn"))


init()
connectMCControl()
startMCControl()
