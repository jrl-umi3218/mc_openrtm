set(CONFIG_FILES
  PDcontroller.in.conf.choreonoid
  robot.in.conf
  robot.in.py
  RobotHardware.in.conf
  sim_mc.in.cnoid
)

foreach(CFG ${CONFIG_FILES})
  string(REPLACE ".in" "" CFG_OUT "${CFG}")
  configure_file(${CFG} ${CMAKE_CURRENT_BINARY_DIR}/${CFG_OUT} @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CFG_OUT}
    DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/JVRC1")
endforeach()

install(FILES
  PDgains_sim.dat
  rtc.conf.choreonoid
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
