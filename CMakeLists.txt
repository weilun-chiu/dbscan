cmake_minimum_required(VERSION 3.11.4)
project(dbscan)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -O2 -g -fopenmp -mavx2")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp -lpthread -mavx2")

set(SRCDIR src)
set(BUILDDIR build)

file(GLOB SRCS ${SRCDIR}/*.cpp)
file(GLOB DEPS ${SRCDIR}/*.h)

set(TARGET dbscan)

add_executable(${TARGET} ${SRCS} main.cpp)
target_include_directories(${TARGET} PRIVATE ${SRCDIR})
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(test test.cpp ${SRCDIR}/point.cpp)
target_include_directories(test PRIVATE ${SRCDIR})
set_target_properties(test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
