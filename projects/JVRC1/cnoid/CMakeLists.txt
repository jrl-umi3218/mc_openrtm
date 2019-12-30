install(FILES
  PDcontroller.conf.choreonoid
  PDgains_sim.dat
  RobotHardware.conf
  robot.conf
  robot.py
  rtc.conf.choreonoid
  sim_mc.cnoid
  sim_mc.py
  DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/JVRC1"
  )

install(FILES
  Virtual-JVRC1-RTC.conf
  DESTINATION ${CHOREONOID_PLUGIN_DIR}/rtc/)

if(NOT EXISTS "${CHOREONOID_PLUGIN_DIR}/rtc/PDcontroller.so")
  if(NOT EXISTS "${HRPSYS_BASE_LIBDIR}/PDcontroller.so")
    message(FATAL_ERROR "PDcontroller.so from hrpsys-base is required to use this project")
  endif()
  install(FILES "${HRPSYS_BASE_LIBDIR}/PDcontroller.so" DESTINATION "${CHOREONOID_PLUGIN_DIR}/rtc/")
endif()
