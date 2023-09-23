cmake_minimum_required(VERSION 3.22)
project(stm32_rfm69 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)

include_directories(.)

add_library(stm32_rfm69
    src/rfm69.c
    include/rfm69.h
)