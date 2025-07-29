find_package(casadi_kin_dyn REQUIRED)
find_package(casadi 3.5.5 REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

add_executable(leg_visualizer
    src/leg_visualizer.cpp
    src/leg_visualizer_node.cpp
)

target_link_libraries(leg_visualizer
    PUBLIC 
    ${catkin_LIBRARIES}
    ${casadi_kin_dyn_LIBRARIES}
    PRIVATE
    rerun_sdk
)

install(TARGETS leg_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
