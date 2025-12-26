project(twistestimator)

# 配置 Valgrind 进行内存泄漏检测
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# ASan 内存检测
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ASAN_FLAGS}")
endif()

# use IWE for event angular velocity estimation
add_definitions(-DUSE_IWE_ESTI)
# add_definitions(
#   -DEIGEN_DONT_ALIGN_STATICALLY
#   -DEIGEN_DONT_VECTORIZE
#   -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
# )

# add_library(spline_lib spline/trajectory.cpp) # spline/twist_trajectory.cpp)
add_library(spline_lib spline/twist_trajectory.cpp)
target_link_libraries(spline_lib 
    ${thirdparty_libraries}
)
# 为 test_estimator 添加 ASan
target_compile_options(spline_lib PRIVATE ${ASAN_OPTIONS})
target_link_options(spline_lib PRIVATE ${ASAN_OPTIONS})

add_library(twistestimator SHARED
    utils/parameter_struct.cpp
    factor/marginalization_factor.cpp
    factor/TwistFactor.h
    estimator/trajectory_estimator2.cpp
    estimator/TwistEstimator.h
    factor/DebugFile.cpp
)

target_link_libraries(twistestimator 
    twistdetector
    spline_lib
    ${thirdparty_libraries}
    ${catkin_LIBRARIES}
)

add_executable(twist_estimator_node twist_estimator_node.cpp)
target_link_libraries(twist_estimator_node
    twistestimator
    twistdetector
    spline_lib
    ${thirdparty_libraries}
    usb-1.0
)