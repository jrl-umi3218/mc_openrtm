#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(IDLS "MCControlService")

foreach(IDL ${IDLS})
  generate_idl_cpp(${IDL} ${CMAKE_CURRENT_SOURCE_DIR})
  generate_idl_python(${IDL} ${CMAKE_CURRENT_SOURCE_DIR})
  install(FILES ${IDL}.idl DESTINATION share/hrpsys/idl)
  set(CPP_IDL_OUT "${CMAKE_CURRENT_BINARY_DIR}/${IDL}SK.cc")
  set(PYTHON_IDL_OUT "${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py")
  set_source_files_properties(FILES ${PYTHON_IDL_OUT} PROPERTIES GENERATED 1)
  install(FILES ${PYTHON_IDL_OUT} DESTINATION "${HRPSYS_BASE_PREFIX}/lib/python2.7/dist-packages/hrpsys")
endforeach()

add_custom_target(generate_mc_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
add_custom_target(generate_mc_idl_python ALL DEPENDS ${ALL_IDL_PYTHON_STUBS})
set(ALL_IDL_CPP_STUBS_ABS)
foreach(STUB ${ALL_IDL_CPP_STUBS})
  list(APPEND ALL_IDL_CPP_STUBS_ABS "${CMAKE_CURRENT_BINARY_DIR}/${STUB}")
endforeach()
set(ALL_IDL_CPP_STUBS ${ALL_IDL_CPP_STUBS_ABS} PARENT_SCOPE)
