find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)

include_directories (${CMAKE_BINARY_DIR}/gui)

set (GUI_SOURCES   main.cpp main_window.cpp seed_selection.cpp)
set (GUI_HEADERS   main_window.h seed_selection.h)
set (GUI_FORMS     main_window.ui)
set (VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (GUI_HEADERS_MOC ${GUI_HEADERS}
              OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
                      -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
QT4_WRAP_UI  (GUI_FORMS_HEADERS ${GUI_FORMS})

include         (${QT_USE_FILE})
add_definitions (${QT_DEFINITIONS} "-DQT_NO_KEYWORDS")

add_executable(gui_random_walker_segmentation
  ${GUI_SOURCES}
  ${GUI_FORMS_HEADERS}
  ${GUI_HEADERS_MOC}
)

target_link_libraries(gui_random_walker_segmentation
  io
  random_walker_segmentation
  ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${TVIEWER_LIBRARIES}
)
