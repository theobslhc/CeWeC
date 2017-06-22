cmake_minimum_required(VERSION 3.0.0)

project(CeWeC)

set(SOURCES_DIR src)
set(HEADERS_DIR include)

file(GLOB_RECURSE srcs ${SOURCES_DIR}/*.cpp)
file(GLOB_RECURSE headers ${HEADERS_DIR}/*.cpp) 

#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_FLAGS "-O3 -march=native -Wall -std=c++11")

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(libLAS)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${libLAS_INCLUDE_DIRS})

include_directories(${HEADERS_DIR})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${libLAS_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(cewec ${srcs} ${headers})

target_link_libraries(cewec ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${libLAS_LIBRARIES})