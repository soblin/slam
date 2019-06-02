project(slam)

cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++11 -Wall -DDEBUG)

#find_package(Boost REQUIRED)
#find_package(Eigen3)

include_directories(
  include/
#  ${EIGEN3_INCLUDE_DIR}
#  ${Boost_INCLUDE_DIR}
  )

file(GLOB lib_sources "lib/geometry/*.cpp" "lib/io/*.cpp" "lib/icp/*.cpp")
file(GLOB lib_headers "include/slam/geoemetry/*.h" "include/slam/io/*.h" "include/slam/icp/*.h")
file(GLOB test_sources "utest/*.cpp")
file(GLOB sources "src/*.cpp")

# libslam
add_library(
  ${PROJECT_NAME}
  SHARED
  ${lib_sources}
  )


# main programs
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")

add_executable(showScans src/showScans.cpp)
target_link_libraries(showScans ${PROJECT_NAME})
add_executable(mapByOdometry src/mapByOdometry.cpp)
target_link_libraries(mapByOdometry ${PROJECT_NAME})

# unit tests
if(NOT without-test)
  enable_testing()
  add_subdirectory(utest)
endif()


# clang format
function(add_sources)
  # check if property "SRCS_LIST" is defined globally
  # and its return value is "is_define"
  get_property(is_defined GLOBAL PROPERTY SRCS_LIST DEFINED)
  if(NOT is_defined)
    define_property(GLOBAL PROPERTY SRCS_LIST
      BRIEF_DOCS "List of source files"
      FULL_DOCS "List of all source files in the entire project")
  endif()
  # make absolute paths
  set(SRCS)
  foreach(s IN LISTS ARGN)
    if(NOT IS_ABSOLUTE "${s}")
      get_filename_component(s "${s}" ABSOLUTE)
    endif()
    list(APPEND SRCS "${s}")
  endforeach()
  # append to global list
  set_property(GLOBAL APPEND PROPERTY SRCS_LIST "${SRCS}")
endfunction()

set(project_sources
  ${lib_sources}
  ${lib_headers}
  ${sources}
  ${test_sources}
  )
add_sources(${project_sources})
get_property(all_sources GLOBAL PROPERTY SRCS_LIST)

# Run clang-format
find_program(CLANG_FORMAT_EXE clang-format)
if(CLANG_FORMAT_EXE)
  add_custom_target(format ALL
    COMMAND clang-format --style=file -i ${all_sources}
    COMMENT "Running clang-format"
    VERBATIM)
endif()

  
