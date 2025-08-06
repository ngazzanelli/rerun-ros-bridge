cmake_minimum_required(VERSION 3.1.0)
project(rerun_ros_bridge)

add_compile_options(-std=c++17)

find_package(casadi_kin_dyn REQUIRED)
find_package(casadi 3.5.5 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
             roscpp
             tf
             std_msgs
             geometry_msgs
             sensor_msgs
)
find_package(XBotInterface REQUIRED)
find_package(pinocchio REQUIRED)


include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/download/0.23.3/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS)

add_executable(rerun_visualizer
    src/visualizer.cpp
    src/visualizer_node.cpp
)

target_include_directories(rerun_visualizer
    PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
)

target_link_libraries(rerun_visualizer
    PUBLIC 
    ${catkin_LIBRARIES}
    ${casadi_kin_dyn_LIBRARIES}
    ${XBotInterface_LIBRARIES}
    PRIVATE
    rerun_sdk
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(TARGETS rerun_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
