cmake_minimum_required(VERSION 3.10)
project(course_project)

set(CMAKE_CXX_STANDARD 14)
find_package(CxxTest REQUIRED)
if(CXXTEST_FOUND)
    include_directories(${CXXTEST_INCLUDE_DIR})
    enable_testing()
    CXXTEST_ADD_TEST(unittest_solver solver_test.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/solver_test.h src/solver.cpp src/stb_image.h)
endif()

add_executable(main src/main.cpp src/solver.cpp src/stb_image_write.h src/stb_image.h)