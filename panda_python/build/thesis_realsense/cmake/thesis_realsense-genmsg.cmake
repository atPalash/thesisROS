# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "thesis_realsense: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(thesis_realsense_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv" NAME_WE)
add_custom_target(_thesis_realsense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thesis_realsense" "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_thesis_realsense_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thesis_realsense" "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_realsense
)
_generate_srv_cpp(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_realsense
)

### Generating Module File
_generate_module_cpp(thesis_realsense
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_realsense
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(thesis_realsense_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(thesis_realsense_generate_messages thesis_realsense_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_cpp _thesis_realsense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_cpp _thesis_realsense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_realsense_gencpp)
add_dependencies(thesis_realsense_gencpp thesis_realsense_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_realsense_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thesis_realsense
)
_generate_srv_eus(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thesis_realsense
)

### Generating Module File
_generate_module_eus(thesis_realsense
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thesis_realsense
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(thesis_realsense_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(thesis_realsense_generate_messages thesis_realsense_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_eus _thesis_realsense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_eus _thesis_realsense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_realsense_geneus)
add_dependencies(thesis_realsense_geneus thesis_realsense_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_realsense_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_realsense
)
_generate_srv_lisp(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_realsense
)

### Generating Module File
_generate_module_lisp(thesis_realsense
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_realsense
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(thesis_realsense_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(thesis_realsense_generate_messages thesis_realsense_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_lisp _thesis_realsense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_lisp _thesis_realsense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_realsense_genlisp)
add_dependencies(thesis_realsense_genlisp thesis_realsense_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_realsense_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thesis_realsense
)
_generate_srv_nodejs(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thesis_realsense
)

### Generating Module File
_generate_module_nodejs(thesis_realsense
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thesis_realsense
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(thesis_realsense_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(thesis_realsense_generate_messages thesis_realsense_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_nodejs _thesis_realsense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_nodejs _thesis_realsense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_realsense_gennodejs)
add_dependencies(thesis_realsense_gennodejs thesis_realsense_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_realsense_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_realsense
)
_generate_srv_py(thesis_realsense
  "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_realsense
)

### Generating Module File
_generate_module_py(thesis_realsense
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_realsense
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(thesis_realsense_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(thesis_realsense_generate_messages thesis_realsense_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/GripperData.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_py _thesis_realsense_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/palash/thesis/thesisROS/panda_python/src/thesis_realsense/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(thesis_realsense_generate_messages_py _thesis_realsense_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_realsense_genpy)
add_dependencies(thesis_realsense_genpy thesis_realsense_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_realsense_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_realsense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_realsense
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(thesis_realsense_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(thesis_realsense_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thesis_realsense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thesis_realsense
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(thesis_realsense_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(thesis_realsense_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_realsense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_realsense
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(thesis_realsense_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(thesis_realsense_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thesis_realsense)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thesis_realsense
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(thesis_realsense_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(thesis_realsense_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_realsense)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_realsense\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_realsense
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(thesis_realsense_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(thesis_realsense_generate_messages_py geometry_msgs_generate_messages_py)
endif()
