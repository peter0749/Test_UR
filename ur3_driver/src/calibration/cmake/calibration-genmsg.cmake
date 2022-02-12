# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "calibration: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(calibration_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv" NAME_WE)
add_custom_target(_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "calibration" "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv" ""
)

get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv" NAME_WE)
add_custom_target(_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "calibration" "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(calibration
  "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
)
_generate_srv_cpp(calibration
  "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
)

### Generating Module File
_generate_module_cpp(calibration
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(calibration_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_cpp _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_cpp _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_gencpp)
add_dependencies(calibration_gencpp calibration_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(calibration
  "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
)
_generate_srv_eus(calibration
  "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
)

### Generating Module File
_generate_module_eus(calibration
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(calibration_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_eus _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_eus _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_geneus)
add_dependencies(calibration_geneus calibration_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(calibration
  "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
)
_generate_srv_lisp(calibration
  "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
)

### Generating Module File
_generate_module_lisp(calibration
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(calibration_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_lisp _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_lisp _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_genlisp)
add_dependencies(calibration_genlisp calibration_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(calibration
  "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
)
_generate_srv_nodejs(calibration
  "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
)

### Generating Module File
_generate_module_nodejs(calibration
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(calibration_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_nodejs _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_nodejs _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_gennodejs)
add_dependencies(calibration_gennodejs calibration_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(calibration
  "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
)
_generate_srv_py(calibration
  "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
)

### Generating Module File
_generate_module_py(calibration
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(calibration_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/GetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_py _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/test/tm700_ws/src/calibration/srv/SetJointPositions.srv" NAME_WE)
add_dependencies(calibration_generate_messages_py _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_genpy)
add_dependencies(calibration_genpy calibration_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(calibration_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(calibration_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(calibration_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(calibration_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(calibration_generate_messages_py std_msgs_generate_messages_py)
endif()
