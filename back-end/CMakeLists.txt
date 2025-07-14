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
    # ${urils_file}
    utils/parameter_struct.cpp
    factor/marginalization_factor.cpp
    factor/TwistFactor.h
    # estimator/trajectory_estimator.cpp
    estimator/trajectory_estimator2.cpp
    # ${spline_file}
    # spline/trajectory.cpp
    # ${factor_file}
    # ${estimator_file}
    estimator/TwistEstimator.h
    factor/DebugFile.cpp
    # estimator/DebugFile.h

)

target_link_libraries(twistestimator 
    twistdetector
    spline_lib
    ${thirdparty_libraries}
    ${catkin_LIBRARIES}
)

# 为 twistestimator 添加 ASan
target_compile_options(twistestimator PRIVATE ${ASAN_OPTIONS})
target_link_options(twistestimator PRIVATE ${ASAN_OPTIONS})


# add_executable(test_estimator test/test_Estimator.cpp)
# target_link_libraries(test_estimator
#     twistestimator
#     twistdetector
#     spline_lib
#     ${thirdparty_libraries}
# )

# # 为 test_estimator 添加 ASan 选项
# target_compile_options(test_estimator PRIVATE -fsanitize=address -fno-omit-frame-pointer)
# target_link_options(test_estimator PRIVATE -fsanitize=address -fno-omit-frame-pointer)


# add_executable(test_estimator2 test/test_Estimator2.cpp)
# target_link_libraries(test_estimator2
#     twistestimator
#     twistdetector
#     spline_lib
#     ${thirdparty_libraries}
# )

# # 为 test_estimator 添加 ASan 选项
# target_compile_options(test_estimator2 PRIVATE -fsanitize=address -fno-omit-frame-pointer)
# target_link_options(test_estimator2 PRIVATE -fsanitize=address -fno-omit-frame-pointer)

# add_executable(test_detector2 test/test_Detector2.cpp)
# target_link_libraries(test_detector2
#     twistestimator
#     twistdetector
#     spline_lib
#     ${thirdparty_libraries}
# )

# # 为 test_estimator 添加 ASan 选项
# target_compile_options(test_detector2 PRIVATE -fsanitize=address -fno-omit-frame-pointer)
# target_link_options(test_detector2 PRIVATE -fsanitize=address -fno-omit-frame-pointer)

# add_executable(twistestimator_iwe test/test_Estimator_IWE.cpp)
# target_link_libraries(twistestimator_iwe
#     twistestimator
#     twistdetector
#     spline_lib
#     ${thirdparty_libraries}
# )

# # 为 twistestimator_iwe 添加 ASan
# target_compile_options(twistestimator_iwe PRIVATE ${ASAN_OPTIONS})
# target_link_options(twistestimator_iwe PRIVATE ${ASAN_OPTIONS})


# ## TEST
# # test_add_bias
# add_executable(test_add_bias test/test_Add_bias.cpp)
# target_link_libraries(test_add_bias
#     twistestimator
#     twistdetector
#     spline_lib
#     ${thirdparty_libraries}
# )
# # 为 test_add_bias 添加 ASan 选项
# target_compile_options(test_add_bias PRIVATE -fsanitize=address -fno-omit-frame-pointer)
# target_link_options(test_add_bias PRIVATE -fsanitize=address -fno-omit-frame-pointer)


# # test_add_bias
# add_executable(test_eigen_matrix test/test_Matrix.cpp)
# target_link_libraries(test_eigen_matrix
#     ${thirdparty_libraries}
# )


add_executable(test_estimator3 test/test_Estimator3.cpp)
target_link_libraries(test_estimator3
    twistestimator
    twistdetector
    spline_lib
    ${thirdparty_libraries}
    usb-1.0
)

# 为 test_estimator 添加 ASan 选项
target_compile_options(test_estimator3 PRIVATE -fsanitize=address -fno-omit-frame-pointer)
target_link_options(test_estimator3 PRIVATE -fsanitize=address -fno-omit-frame-pointer)