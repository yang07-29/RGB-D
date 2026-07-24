cmake_minimum_required(VERSION 3.20)
project(rgbd_odometry_cpp VERSION 0.1.0 LANGUAGES CXX)

option(RGBD_BUILD_APPS "Build standalone odometry executable and CTest target" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs)

add_library(rgbd_core
    src/geometry.cpp
    src/metrics.cpp
    src/rgbd.cpp
    src/tum.cpp
)
target_include_directories(rgbd_core PUBLIC include)
target_link_libraries(rgbd_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
target_compile_definitions(rgbd_core PUBLIC RGBD_OPENCV_VERSION="${OpenCV_VERSION}")

if(MSVC)
    target_compile_options(rgbd_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(rgbd_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(RGBD_BUILD_APPS)
    add_executable(rgbd_odometry src/odometry_main.cpp)
    target_link_libraries(rgbd_odometry PRIVATE rgbd_core)
    if(WIN32)
        target_link_libraries(rgbd_odometry PRIVATE psapi)
    endif()

    include(CTest)
    if(BUILD_TESTING)
        add_executable(rgbd_cpp_tests tests/test_core.cpp)
        target_link_libraries(rgbd_cpp_tests PRIVATE rgbd_core)
        add_test(NAME rgbd_cpp_tests COMMAND rgbd_cpp_tests)
    endif()
endif()
