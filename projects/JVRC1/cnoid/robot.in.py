#!/usr/bin/python
# configuration of JVRC1(robot)

import os
import sys
import math

# path
sys.path.append("@HRPSYS_BASE_PREFIX@/lib/python2.7/dist-packages/hrpsys")
sys.path.append(os.getcwd())

# config
isDemoMode = False
use_istabilizer = False
exec_pattern_with_st = True
save_log = False
log_max_length = 400*200
record_log = False
walk_distance = 1.0
arc_x = 0.0
arc_y = 10.0
arc_th = 60.0/180.0*math.pi

# robot info
nsport = 2809
nshost = "localhost"

modelName = 'JVRC1'
url = 'file://' + \
    '@OPENHRP3_1_PREFIX@/share/OpenHRP-3.1/robot/JVRC1/@JVRC1_MODEL_NAME@.wrl'

# pose parameters
timeToHalfsitPose = 3.0  # [sec]
halfsitPose = [0, -0.76, -22.02, 41.29, -18.75, -0.45,
               0, 1.15, -21.89, 41.21, -18.74, -1.10,
               8, 0,
               0, 0,
               -3, -10, 0, -30, 0, 0, 0,
               0, 0,
               -3, 10, 0, -30, 0, 0, 0,
               0, 0]
dof = len(halfsitPose)
timeToInitialPose = 3.0  # [sec]
initialPose = [0] * dof
initialPose[17] = -10
initialPose[26] = 10
initialPose[23] = 10
initialPose[32] = -10
