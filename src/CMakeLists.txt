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
target_link_libraries(MCControl PUBLIC mc_rtc::mc_control)
target_include_directories(MCControl PRIVATE "${PROJECT_BINARY_DIR}/idl")

add_executable(MCControlComp MCControlComp.cpp)
target_link_libraries(MCControlComp PUBLIC MCControl)

install(TARGETS MCControl MCControlComp
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
