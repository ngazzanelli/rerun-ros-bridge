find_package(casadi_kin_dyn REQUIRED)
find_package(casadi 3.5.5 REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

add_executable(rerun_visualizer
    src/visualizer.cpp
    src/visualizer_node.cpp
)

target_link_libraries(rerun_visualizer
    PUBLIC 
    ${catkin_LIBRARIES}
    ${casadi_kin_dyn_LIBRARIES}
    PRIVATE
    rerun_sdk
)

install(TARGETS rerun_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
