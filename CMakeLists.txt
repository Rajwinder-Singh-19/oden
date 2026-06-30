cmake_minimum_required(VERSION 3.30 FATAL_ERROR)

project(oden LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(COMPILE_OPTIONS "-Wall -Werror -Wextra -Wpedantic -Warray-bounds")

add_executable(App)
target_sources(App PRIVATE src/main.cpp)
target_include_directories(App PRIVATE include)