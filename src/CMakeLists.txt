#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

foreach(STUB ${ALL_IDL_CPP_STUBS})
  set_source_files_properties(${STUB} PROPERTIES GENERATED 1 COMPILE_FLAGS -Wno-error)
endforeach()

set(MCControl_SRC
      MCControlServiceSVC_impl.cpp
      MCControl.cpp
      ${ALL_IDL_CPP_STUBS}
)

set(MCControl_HDR
      MCControlServiceSVC_impl.h
      MCControl.h
)


include_directories(${CMAKE_BINARY_DIR}/idl)

add_library(MCControl SHARED ${MCControl_SRC} ${MCControl_HDR})
add_dependencies(MCControl generate_mc_idl_cpp)
set_target_properties(MCControl PROPERTIES PREFIX "")
pkg_config_use_dependency(MCControl hrpsys-base)
if(NOT TARGET hrpIo::hrpIo)
  find_path(hrpIo_INCLUDE_DIR NAMES iob.h HINTS ${HRPSYS_BASE_PREFIX}/include/hrpsys/io)
  find_path(hrpIo_LIBRARY NAMES hrpIo HINTS ${HRPSYS_BASE_PREFIX})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(hrpIo DEFAULT_MSG REQUIRED hrpIo_LIBRARY hrpIo_INCLUDE_DIR)
  mark_as_advanced(hrpIo_INCLUDE_DIR hrpIo_LIBRARY)
  add_library(hrpIo::hrpIo INTERFACE IMPORTED GLOBAL)
  set_target_properties(hrpIo::hrpIo PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${hrpIo_INCLUDE_DIR}
    INTERFACE_LINK_LIBRARIES ${hrpIo_LIBRARY}
  )
endif()
target_link_libraries(MCControl PUBLIC mc_rtc::mc_control hrpIo)
target_include_directories(MCControl PRIVATE "${PROJECT_BINARY_DIR}/idl")
if(REAL_ROBOT)
  message(STATUS "Building for REAL_ROBOT")
  target_compile_definitions(MCControl PRIVATE USE_IOB)
endif()

add_executable(MCControlComp MCControlComp.cpp)
target_link_libraries(MCControlComp PUBLIC MCControl)

install(TARGETS MCControl MCControlComp
  RUNTIME DESTINATION "${HRPSYS_BASE_PREFIX}/bin"
  LIBRARY DESTINATION "${HRPSYS_BASE_PREFIX}/lib"
  ARCHIVE DESTINATION "${HRPSYS_BASE_PREFIX}/lib"
)
