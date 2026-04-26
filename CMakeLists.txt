cmake_minimum_required(VERSION 3.23)

project(VehiclePathPlanning)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (MSVC)
    add_compile_options(/Zc:__cplusplus)
endif()

add_executable(VehiclePathPlanning)

include(FetchContent)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0  # Use a specific version tag
)

FetchContent_MakeAvailable(eigen)


target_sources(VehiclePathPlanning
  PRIVATE
    src/main.cpp
)


target_link_libraries(VehiclePathPlanning
  PRIVATE
    eigen
)
