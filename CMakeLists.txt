cmake_minimum_required(VERSION 3.25)
project(antsdr_libiio)

set(CMAKE_CXX_STANDARD 17)

find_package(Python3 COMPONENTS Development NumPy)
if(Python3_FOUND)
    message(STATUS "Python3 found: ${Python3_VERSION}")
    include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Python3 not found")
endif()

add_executable(antsdr_libiio main.cpp antsdrDevice.hpp antsdrDevice.cpp)

target_link_libraries(antsdr_libiio iio pthread Python3::Python Python3::NumPy)