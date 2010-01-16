SET(SRC_CPP_FILES
  
)

SET(INTERNAL_CPP_FILES
  berryObjectBrowserView.cpp
  berryObjectItem.cpp
  berryQtObjectTableModel.cpp
)

SET(UI_FILES
  src/internal/berryQtObjectBrowserView.ui
)

SET(MOC_H_FILES
  src/internal/berryObjectBrowserView.h
  src/internal/berryQtObjectTableModel.h
)

SET(RESOURCE_FILES
  resources/ObjectBrowser.png
)

SET(RES_FILES
  resources/blueberry_ui_qt_objectinspector.qrc
)

SET(CPP_FILES manifest.cpp)

foreach(file ${SRC_CPP_FILES})
  SET(CPP_FILES ${CPP_FILES} src/${file})
endforeach(file ${SRC_CPP_FILES})

foreach(file ${INTERNAL_CPP_FILES})
  SET(CPP_FILES ${CPP_FILES} src/internal/${file})
endforeach(file ${INTERNAL_CPP_FILES})

