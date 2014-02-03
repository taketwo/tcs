set(SUBSYS_NAME tviewer)

include_directories(${CMAKE_SOURCE_DIR}/include/${SUBSYS_NAME})

add_library(tviewer
  tviewer.cpp
  utils.cpp
  visualization_object.cpp
  normal_cloud_object.cpp
  poly_data_object.cpp
)
target_link_libraries(tviewer ${PCL_LIBRARIES})
