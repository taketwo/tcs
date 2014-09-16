cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(TCS)
find_package(PCL 1.7.2 REQUIRED)
set(CMAKE_BUILD_TYPE "Release")

option(WITH_QT_GUI "Build Qt GUI application." ON)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# add include folder to the list include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

add_definitions(-std=c++11)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-fcolor-diagnostics)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()

#...: enable all but certain warnings
add_definitions(-Wall)
add_definitions(-Wno-unknown-pragmas)
add_definitions(-Wno-deprecated)
add_definitions(-fpermissive)

# Documentation
set(DOXYFILE_LATEX false)
include(UseDoxygen)

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# TViewer
add_subdirectory(third-party/tviewer)
include_directories(${TVIEWER_INCLUDE_DIRS})
add_definitions(${TVIEWER_DEFINITIONS})

# Hungarian
add_subdirectory(src/hungarian)

add_library(random_walker_segmentation
  src/random_walker_segmentation.cpp
)
target_link_libraries(random_walker_segmentation
  ${PCL_LIBRARIES}
)
set_target_properties(random_walker_segmentation
  PROPERTIES COMPILE_DEFINITIONS "PCL_ONLY_CORE_POINT_TYPES"
)

add_library(io
  src/io.cpp
)
target_link_libraries(io
  ${PCL_LIBRARIES}
)

add_executable(app_random_walker_segmentation
  src/random_walker_segmentation_app.cpp
)
target_link_libraries(app_random_walker_segmentation
  ${PCL_LIBRARIES}
  ${TVIEWER_LIBRARIES}
  random_walker_segmentation
  io
)

add_executable(view_point_cloud_graph
  src/view_point_cloud_graph.cpp
)
target_link_libraries(view_point_cloud_graph
  ${PCL_LIBRARIES}
  ${TVIEWER_LIBRARIES}
  io
)

add_executable(segmentation_evaluation
  src/segmentation_evaluation.cpp
)
target_link_libraries(segmentation_evaluation
  ${PCL_LIBRARIES}
  ${TVIEWER_LIBRARIES}
  hungarian
)

add_executable(create_seeds
  src/create_seeds.cpp
)
target_link_libraries(create_seeds
  ${PCL_LIBRARIES}
)

if (WITH_QT_GUI)
  add_subdirectory(gui)
endif()
