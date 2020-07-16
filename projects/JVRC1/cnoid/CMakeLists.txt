set(CONFIG_FILES
  PDcontroller.in.conf.choreonoid
  robot.in.conf
  robot.in.py
  RobotHardware.in.conf
  rtc.in.conf.choreonoid
  sim_mc.in.cnoid
  sim_mc_wall.in.cnoid
)

# Generate floating cnoid files for floating base model
set(JVRC1_MODEL_NAME "main")
foreach(CFG ${CONFIG_FILES})
  string(REPLACE ".in" "" CFG_OUT "${CFG}")
  configure_file(${CFG} ${CMAKE_CURRENT_BINARY_DIR}/${CFG_OUT} @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CFG_OUT}
    DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/JVRC1")
endforeach()

# Generate cnoid files for fixed-based model 
set(JVRC1_MODEL_NAME "main_fixed")
configure_file(sim_mc_fixed.in.cnoid ${CMAKE_CURRENT_BINARY_DIR}/sim_mc_fixed.cnoid @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sim_mc_fixed.cnoid 
  DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/JVRC1"
)

install(FILES
  PDgains_sim.dat
  sim_mc.py
  DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/JVRC1"
  )
install(PROGRAMS
  clear-omninames.sh
  DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/JVRC1"
  )

install(FILES
  Virtual-JVRC1-RTC.conf
  DESTINATION "${CHOREONOID_PLUGIN_DIR}/rtc/")

if(NOT EXISTS "${CHOREONOID_PLUGIN_DIR}/rtc/PDcontroller${CMAKE_SHARED_LIBRARY_SUFFIX}")
  if(NOT EXISTS "${HRPSYS_BASE_LIBDIR}/PDcontroller${CMAKE_SHARED_LIBRARY_SUFFIX}")
    message(FATAL_ERROR "PDcontroller${CMAKE_SHARED_LIBRARY_SUFFIX} from hrpsys-base is required to use this project")
  endif()
  install(FILES "${HRPSYS_BASE_LIBDIR}/PDcontroller${CMAKE_SHARED_LIBRARY_SUFFIX}" DESTINATION "${CHOREONOID_PLUGIN_DIR}/rtc/")
endif()
