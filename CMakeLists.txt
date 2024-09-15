cmake_minimum_required(VERSION 3.27)
set(PROJECT_NAME Verlet)
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release) 

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SOURCES main.cpp)


set(SPATIAL_HASH_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Spatial_Hash")
set(PHYSICS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Physics")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


include_directories(
  ${SPATIAL_HASH_INCLUDE_DIR}
  ${PHYSICS_INCLUDE_DIR}
  ${SHADER_INCLUDE_DIR}
)




# Detect and add SFML
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})
find_package(SFML 2.6 REQUIRED COMPONENTS  graphics window )

add_executable(${PROJECT_NAME} 
   main.cpp
)
target_link_libraries(${PROJECT_NAME}  sfml-window sfml-graphics)
# set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
if (UNIX)
   target_link_libraries(${PROJECT_NAME} pthread)
endif (UNIX)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/SHADER DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
get_filename_component(SHADER_ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/SHADER" ABSOLUTE)
 target_compile_definitions(Verlet  PRIVATE SHADER_DIR="${SHADER_ABSOLUTE_PATH}")
