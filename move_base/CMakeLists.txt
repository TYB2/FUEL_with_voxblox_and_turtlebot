cmake_minimum_required(VERSION 3.0.2)
project(move_base)


find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

add_message_files(
  FILES
  TrackerStatus.msg
  OdomData.msg
)

add_service_files(
  FILES
  TrackerPath.srv
  PlannerGoal.srv
  ClearPath.srv
  GetTrajectory.srv
  TurtlebotStatus.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(

    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES move_base
    CATKIN_DEPENDS
        # message_generation
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(move_base
  src/move_base.cpp
)
target_link_libraries(move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_base_node
  src/move_base_node.cpp
)
add_dependencies(move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_node move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base)

# install(
#     TARGETS
#         move_base_node
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#     TARGETS
#         move_base
#     DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

add_library(pid_controller  src/pid_controller.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_library(turtlebot_global_planner src/turtlebot_global_planner.cpp)
target_link_libraries(turtlebot_global_planner ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
# add_dependencies(move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(global_planner_node src/global_planner_node.cpp)
target_link_libraries(global_planner_node turtlebot_global_planner)

add_library(turtlebot_local_tracker src/turtlebot_local_tracker.cpp)
target_link_libraries(turtlebot_local_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
# add_dependencies(move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(turtlebot_local_tracker_with_pid src/turtlebot_local_tracker_with_pid.cpp)
target_link_libraries(turtlebot_local_tracker_with_pid pid_controller ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
# add_dependencies(move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(local_tracker_node src/local_tracker_node.cpp)
target_link_libraries(local_tracker_node turtlebot_local_tracker_with_pid turtlebot_local_tracker )

add_library(trajectory_recoder src/trajectory_recoder.cpp)
target_link_libraries(trajectory_recoder ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(trajectory_recode_node src/trajectory_recode_node.cpp)
target_link_libraries(trajectory_recode_node trajectory_recoder)

add_executable(planner_test_node src/planner_test_node.cpp)
target_link_libraries(planner_test_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
