cmake_minimum_required(VERSION 3.22)
project(stm32_rfm69 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)

file(GLOB_RECURSE SOURCES "src/*.c")

include_directories(.)

add_library(stm32_rfm69 ${SOURCES})
include_directories(./include)
