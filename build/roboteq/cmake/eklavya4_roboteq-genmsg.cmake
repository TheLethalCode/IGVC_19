# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "eklavya4_roboteq: 2 messages, 1 services")

set(MSG_I_FLAGS "-Ieklavya4_roboteq:/home/mechanical/igvc_19/src/roboteq/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(eklavya4_roboteq_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv" NAME_WE)
add_custom_target(_eklavya4_roboteq_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eklavya4_roboteq" "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv" ""
)

get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/control.msg" NAME_WE)
add_custom_target(_eklavya4_roboteq_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eklavya4_roboteq" "/home/mechanical/igvc_19/src/roboteq/msg/control.msg" ""
)

get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg" NAME_WE)
add_custom_target(_eklavya4_roboteq_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eklavya4_roboteq" "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eklavya4_roboteq
)
_generate_msg_cpp(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Services
_generate_srv_cpp(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Module File
_generate_module_cpp(eklavya4_roboteq
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eklavya4_roboteq
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(eklavya4_roboteq_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(eklavya4_roboteq_generate_messages eklavya4_roboteq_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_cpp _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/control.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_cpp _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_cpp _eklavya4_roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eklavya4_roboteq_gencpp)
add_dependencies(eklavya4_roboteq_gencpp eklavya4_roboteq_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eklavya4_roboteq_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eklavya4_roboteq
)
_generate_msg_eus(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Services
_generate_srv_eus(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Module File
_generate_module_eus(eklavya4_roboteq
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eklavya4_roboteq
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(eklavya4_roboteq_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(eklavya4_roboteq_generate_messages eklavya4_roboteq_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_eus _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/control.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_eus _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_eus _eklavya4_roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eklavya4_roboteq_geneus)
add_dependencies(eklavya4_roboteq_geneus eklavya4_roboteq_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eklavya4_roboteq_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eklavya4_roboteq
)
_generate_msg_lisp(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Services
_generate_srv_lisp(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Module File
_generate_module_lisp(eklavya4_roboteq
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eklavya4_roboteq
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(eklavya4_roboteq_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(eklavya4_roboteq_generate_messages eklavya4_roboteq_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_lisp _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/control.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_lisp _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_lisp _eklavya4_roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eklavya4_roboteq_genlisp)
add_dependencies(eklavya4_roboteq_genlisp eklavya4_roboteq_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eklavya4_roboteq_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eklavya4_roboteq
)
_generate_msg_nodejs(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Services
_generate_srv_nodejs(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Module File
_generate_module_nodejs(eklavya4_roboteq
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eklavya4_roboteq
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(eklavya4_roboteq_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(eklavya4_roboteq_generate_messages eklavya4_roboteq_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_nodejs _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/control.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_nodejs _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_nodejs _eklavya4_roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eklavya4_roboteq_gennodejs)
add_dependencies(eklavya4_roboteq_gennodejs eklavya4_roboteq_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eklavya4_roboteq_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eklavya4_roboteq
)
_generate_msg_py(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Services
_generate_srv_py(eklavya4_roboteq
  "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eklavya4_roboteq
)

### Generating Module File
_generate_module_py(eklavya4_roboteq
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eklavya4_roboteq
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(eklavya4_roboteq_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(eklavya4_roboteq_generate_messages eklavya4_roboteq_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/srv/SetSpeed.srv" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_py _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/control.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_py _eklavya4_roboteq_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mechanical/igvc_19/src/roboteq/msg/diagnose_msg.msg" NAME_WE)
add_dependencies(eklavya4_roboteq_generate_messages_py _eklavya4_roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eklavya4_roboteq_genpy)
add_dependencies(eklavya4_roboteq_genpy eklavya4_roboteq_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eklavya4_roboteq_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eklavya4_roboteq)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eklavya4_roboteq
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(eklavya4_roboteq_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eklavya4_roboteq)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eklavya4_roboteq
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(eklavya4_roboteq_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eklavya4_roboteq)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eklavya4_roboteq
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(eklavya4_roboteq_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eklavya4_roboteq)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eklavya4_roboteq
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(eklavya4_roboteq_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eklavya4_roboteq)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eklavya4_roboteq\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eklavya4_roboteq
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(eklavya4_roboteq_generate_messages_py std_msgs_generate_messages_py)
endif()
