project(twistdetector)


set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

file(GLOB_RECURSE event_flow_detector_file "event_flow_detector/*.cpp" "front-end/event_flow_detector/*.h")
file(GLOB_RECURSE radar_ego_velocity_estimator_file "radar_ego_velocity_estimator/*.cpp" "front-end/radar_ego_velocity_estimator/*.h")


add_library(twistdetector SHARED
    ${event_flow_detector_file}
    ${radar_ego_velocity_estimator_file}
)
target_link_libraries(twistdetector
${thirdparty_libraries}
)

# add_executable(test_detector test/testEventDetector.cpp)
# target_link_libraries(  
#   test_detector
#   twistdetector
#   ${thirdparty_libraries}
# )
