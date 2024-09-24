cmake_minimum_required(VERSION 3.8)
project(bt_executer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(cnr_param REQUIRED)
find_package(trajectory_loader REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(force_sensor REQUIRED)
find_package(yolov8 REQUIRED)
find_package(tf REQUIRED)

set(EXECUTER_DEPS
    cnr_param::cnr_param
    )
set(ROS_EXECUTER_DEPS
    rclcpp
    std_msgs
    behaviortree_ros2
    btcpp_ros2_interfaces
    )
set(SKILL_LIBRARY_DEPS
    rclcpp
    geometry_msgs
    behaviortree_ros2
    btcpp_ros2_interfaces
    trajectory_loader
    tf2
    tf2_ros
    std_srvs
    control_msgs
    rclcpp_action
    force_sensor
    yolov8
    tf
    )

# =====
# BUILD
# =====
# Build the skills library
add_library(move_to_plugin SHARED
    src/skills/move_to_skill.cpp
    )
target_link_libraries(move_to_plugin ${EXECUTER_DEPS})
target_compile_definitions(move_to_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(move_to_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(move_to_plugin ${SKILL_LIBRARY_DEPS})

add_library(trajectory_loader_plugin SHARED
    src/skills/trajectory_loader_skill.cpp
    )
target_link_libraries(trajectory_loader_plugin ${EXECUTER_DEPS})
target_compile_definitions(trajectory_loader_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(trajectory_loader_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(trajectory_loader_plugin ${SKILL_LIBRARY_DEPS})

add_library(trigger_service_plugin SHARED
    src/skills/trigger_service_skill.cpp
    )
target_link_libraries(trigger_service_plugin ${EXECUTER_DEPS})
target_compile_definitions(trigger_service_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(trigger_service_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(trigger_service_plugin ${SKILL_LIBRARY_DEPS})

add_library(set_bool_service_plugin SHARED
    src/skills/set_bool_service_skill.cpp
    )
target_link_libraries(set_bool_service_plugin ${EXECUTER_DEPS})
target_compile_definitions(set_bool_service_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(set_bool_service_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(set_bool_service_plugin ${SKILL_LIBRARY_DEPS})

add_library(wait_plugin SHARED
    src/skills/wait_skill.cpp
    )
target_link_libraries(wait_plugin ${EXECUTER_DEPS})
target_compile_definitions(wait_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(wait_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(wait_plugin ${SKILL_LIBRARY_DEPS})

add_library(check_touch_service_plugin SHARED
  src/skills/check_touch_service_skill.cpp
	)
target_link_libraries(check_touch_service_plugin ${EXECUTER_DEPS})	
target_compile_definitions(check_touch_service_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(check_touch_service_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(check_touch_service_plugin ${SKILL_LIBRARY_DEPS})

add_library(store_actual_position_plugin SHARED
    src/skills/store_actual_position_skill.cpp
    )
target_link_libraries(store_actual_position_plugin ${EXECUTER_DEPS})
target_compile_definitions(store_actual_position_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(store_actual_position_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(store_actual_position_plugin ${SKILL_LIBRARY_DEPS})

add_library(object_detection_plugin SHARED
    src/skills/object_detection_skill.cpp
    )
target_link_libraries(object_detection_plugin ${EXECUTER_DEPS})
target_compile_definitions(object_detection_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(object_detection_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(object_detection_plugin ${SKILL_LIBRARY_DEPS})

add_library(store_actual_position_and_compute_delta_plugin SHARED
    src/skills/store_actual_position_and_compute_delta_skill.cpp
    )
target_link_libraries(store_actual_position_and_compute_delta_plugin ${EXECUTER_DEPS})
target_compile_definitions(store_actual_position_and_compute_delta_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(store_actual_position_and_compute_delta_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(store_actual_position_and_compute_delta_plugin ${SKILL_LIBRARY_DEPS})

add_library(tf_publish_service_plugin SHARED
    src/skills/tf_publish_service_skill.cpp
    )
target_link_libraries(tf_publish_service_plugin ${EXECUTER_DEPS})
target_compile_definitions(tf_publish_service_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(tf_publish_service_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(tf_publish_service_plugin ${SKILL_LIBRARY_DEPS})

add_library(compute_and_publish_center_plugin SHARED
    src/skills/compute_and_publish_center_skill.cpp
    )
target_link_libraries(compute_and_publish_center_plugin ${EXECUTER_DEPS})
target_compile_definitions(compute_and_publish_center_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(compute_and_publish_center_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(compute_and_publish_center_plugin ${SKILL_LIBRARY_DEPS})

# Build the skill executer node
add_executable(bt_executer_node
    src/bt_executer_node.cpp
    )
target_link_libraries(bt_executer_node ${EXECUTER_DEPS})
ament_target_dependencies(bt_executer_node ${ROS_EXECUTER_DEPS})

# =======
# INSTALL
# =======
install(TARGETS
    move_to_plugin
    trajectory_loader_plugin
    trigger_service_plugin
    set_bool_service_plugin
    check_touch_service_plugin
    store_actual_position_plugin
    object_detection_plugin
    store_actual_position_and_compute_delta_plugin
    tf_publish_service_plugin
    compute_and_publish_center_plugin
    wait_plugin
    bt_executer_node
    DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY include
    DESTINATION include/)

install(DIRECTORY launch config trees
    DESTINATION share/${PROJECT_NAME})

# ====
# TEST
# ====
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_cmake_gmock REQUIRED)
#   # ament_add_gtest(some_test <test_sources>)
#   # ament_add_gmock(some_test <test_sources>)
# endif()

ament_package()
