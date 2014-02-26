macro(TEST_ADD _name)
  set(options)
  set(one_value_args)
  set(multi_value_args LINK_WITH)
  cmake_parse_arguments(TEST_ADD "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})
  set(_executable test_${_name})
  add_executable(${_executable} ${_executable}.cpp)
  target_link_libraries(${_executable} ${GTEST_LIBRARIES} ${TEST_ADD_LINK_WITH} pthread)
  add_test(NAME ${_name} COMMAND ${_executable})
  add_dependencies(tests ${_executable})
endmacro(TEST_ADD)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

get_filename_component(TEST_WITH_DATA_TEST_DIR "data" ABSOLUTE)
configure_file(test_with_data_settings.h.in test/test_with_data_settings.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

add_custom_target(tests "${CMAKE_CTEST_COMMAND}" "-V" VERBATIM)

TEST_ADD(random_walker
)
TEST_ADD(point_cloud_graph
         LINK_WITH ${PCL_LIBRARIES}
)
TEST_ADD(weight_computer
         LINK_WITH ${PCL_LIBRARIES}
)
TEST_ADD(graph_common
         LINK_WITH ${PCL_LIBRARIES}
)
TEST_ADD(graph_builders
         LINK_WITH ${PCL_LIBRARIES}
)
