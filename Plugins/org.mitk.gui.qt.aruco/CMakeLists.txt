project(org_mitk_gui_qt_aruco)

MACRO_CREATE_MITK_CTK_PLUGIN(
  EXPORT_DIRECTIVE ARUCO_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkIOExt MitkQtWidgetsExt MitkOpenCVVideoSupport MitkIGTUI
)

find_package(aruco REQUIRED )
include_directories(${aruco_INCLUDE_DIRS})

target_link_libraries(${PLUGIN_TARGET} ${aruco_LIBS})
