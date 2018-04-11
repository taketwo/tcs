find_package (VTK REQUIRED)

if(VTK_VERSION VERSION_LESS "6.0")
  message(STATUS "VTK version is 5 or below, therefore using Qt4")
  find_package(Qt4 REQUIRED)
  include(${QT_USE_FILE})
  include(${VTK_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  list(APPEND VTK_LIBRARIES QVTK)
else()
  message(STATUS "VTK version is 6 or above, therefore using Qt5")
  find_package(Qt5Widgets REQUIRED)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(${QT_DEFINITIONS} "-DQT_NO_KEYWORDS")

set(GUI_SOURCES   main.cpp main_window.cpp seed_selection.cpp)
set(GUI_HEADERS   main_window.h seed_selection.h)
set(GUI_FORMS     main_window.ui)

qt_wrap_ui(GUI_FORMS_HEADERS ${GUI_FORMS})

add_executable(gui_random_walker_segmentation
  ${GUI_SOURCES}
  ${GUI_FORMS_HEADERS}
)

target_link_libraries(gui_random_walker_segmentation
  io
  random_walker_segmentation
  ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  tviewer
)
