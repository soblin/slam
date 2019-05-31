cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)
add_executable(sensor_data_reader_test sensor_data_reader.cpp)
target_link_libraries(sensor_data_reader_test
  slam
  gtest
  gtest_main
  pthread
  )

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_test(
  NAME sensor-data-reader
  COMMAND $<TARGET_FILE:sensor_data_reader_test>
  )
