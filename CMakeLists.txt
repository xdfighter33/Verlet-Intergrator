cmake_minimum_required(VERSION 3.16)
project(CMakeSFMLProject LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

add_executable(CMakeSFMLProject main.cpp)


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
get_filename_component(SHADER_ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Shaders" ABSOLUTE)
target_compile_definitions(Ray_Marching PRIVATE SHADER_DIR="${SHADER_ABSOLUTE_PATH}")



target_link_libraries(CMakeSFMLProject PRIVATE sfml-graphics)
target_compile_features(CMakeSFMLProject PRIVATE cxx_std_17)



install(TARGETS CMakeSFMLProject)