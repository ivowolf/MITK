# The project name must correspond to the directory name of your plug-in
# and must not contain periods.
project(org_mitk_gui_qt_measurementtoolbox)

MACRO_CREATE_MITK_CTK_PLUGIN(
  EXPORT_DIRECTIVE MITK_QT_MEASUREMENTTOOLBOX
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkImageStatistics MitkPlanarFigure
)
