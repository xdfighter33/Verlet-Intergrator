cmake_minimum_required(VERSION 3.16)

project(CMakeSFMLProject LANGUAGES CXX)


set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fno-omit-frame-pointer")

include(FetchContent)

FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)

FetchContent_MakeAvailable(SFML)

set(SPATIAL_HASH_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Spatial_Hash")
set(PHYSICS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Physics")


include_directories(
  ${SPATIAL_HASH_INCLUDE_DIR}
  ${PHYSICS_INCLUDE_DIR}
  ${SHADER_INCLUDE_DIR}
)


add_executable(CMakeSFMLProject main.cpp)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/SHADER DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

get_filename_component(SHADER_ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/SHADER" ABSOLUTE)

target_compile_definitions(CMakeSFMLProject PRIVATE SHADER_DIR="${SHADER_ABSOLUTE_PATH}")

target_link_libraries(CMakeSFMLProject PRIVATE sfml-graphics)

target_compile_features(CMakeSFMLProject PRIVATE cxx_std_17)



install(TARGETS CMakeSFMLProject)