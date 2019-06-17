cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

# gtest
add_library(gtest STATIC IMPORTED)
set_property(
  TARGET gtest
  PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../lib/googletest/googletest/build/lib/libgtest.a
  )

add_library(gtest_main STATIC IMPORTED)
set_property(
  TARGET gtest_main
  PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../lib/googletest/googletest/build/lib/libgtest_main.a
  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/googletest/googletest/include)

add_executable(sensor_data_reader_test sensor_data_reader.cpp)
target_link_libraries(sensor_data_reader_test
  slam
  gtest
  gtest_main
  pthread
  )
add_test(
  NAME sensor-data-reader
  COMMAND $<TARGET_FILE:sensor_data_reader_test>
  )

add_executable(pose2d_test pose2d.cpp)
target_link_libraries(pose2d_test
  slam
  gtest
  gtest_main
  pthread)
add_test(
  NAME pose2d_test
  COMMAND $<TARGET_FILE:pose2d_test>
  )

add_executable(scanpoint2d_test scanpoint2d.cpp)
target_link_libraries(scanpoint2d_test
  slam
  gtest
  gtest_main
  pthread)
add_test(
  NAME scanpoint2d_test
  COMMAND $<TARGET_FILE:scanpoint2d_test>
  )

add_executable(data_associatorls_test data_associatorls.cpp)
target_link_libraries(data_associatorls_test
  slam
  gtest
  gtest_main
  pthread)
add_test(
  NAME data_associatorls
  COMMAND $<TARGET_FILE:data_associatorls_test>
  )

add_executable(costfunction_ed_test costfunction_ed.cpp)
target_link_libraries(costfunction_ed_test
  slam
  gtest
  gtest_main
  pthread)
add_test(
  NAME costfunction_ed_test
  COMMAND $<TARGET_FILE:costfunction_ed_test>
  )

add_executable(scanpointresampler_test scanpointresampler.cpp)
target_link_libraries(scanpointresampler_test
  slam
  gtest
  gtest_main
  pthread)
add_test(
  NAME scanpointresampler_test
  COMMAND $<TARGET_FILE:scanpointresampler_test>
  )

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

