project(org_mitk_gui_qt_aruco)

include_directories("/home/riecker/Development/libs/aruco-1.2.5/src")
include_directories("/home/riecker/Development/aruco/aruco-1.2.5/src")

LINK_DIRECTORIES("/usr/local/lib")

MACRO_CREATE_MITK_CTK_PLUGIN(
  EXPORT_DIRECTIVE ARUCO_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkIOExt MitkQtWidgetsExt MitkOpenCVVideoSupport MitkIGTUI
)

find_package(aruco REQUIRED )
include_directories(${aruco_INCLUDE_DIRS})
LINK_DIRECTORIES(${aruco_LIB_DIR})
target_link_libraries(${PLUGIN_TARGET} ${aruco_LIBS})
#SET_TARGET_PROPERTIES(${PLUGIN_TARGET} PROPERTIES LINK_LIBRARIES ${aruco_LIB_DIR})

### OLD STUFF MAYBE NEEDED FOR WINDOWS & MAC ###
#set(ArUco_LIBRARY_DIR "/home/riecker/Development/aruco/aruco-1.2.5-bin/src" CACHE FILEPATH "Path of ArUco lib.")
#set(ArUco_INCLUDE_DIR "/home/riecker/Development/aruco/aruco-1.2.5/src" CACHE FILEPATH "Path of ArUco lib.")
#set(ArUco_LIBS "/home/riecker/Development/aruco/aruco-1.2.5-bin/src/libaruco.so") # ?

#include_directories("/home/riecker/Development/aruco/aruco-1.2.5/src")

#link_directories("/home/riecker/Development/aruco/aruco-1.2.5-bin/src")
#link_directories("/home/riecker/Development/aruco/aruco-1.2.5-bin/src")
#link_directories("/usr/local/lib")

#found in findAruco.cmake:
#set(CMAKE_MODULE_PATH /usr/local/lib/cmake/ )
#set(aruco_LIBS opencv_videostab;opencv_video;opencv_ts;opencv_superres;opencv_stitching;opencv_photo;opencv_ocl;opencv_objdetect;opencv_nonfree;opencv_ml;opencv_legacy;opencv_imgproc;opencv_highgui;opencv_gpu;opencv_flann;opencv_features2d;opencv_core;opencv_contrib;opencv_calib3d aruco)

#list(APPEND ALL_LIBRARIES "/home/riecker/Development/aruco/aruco-1.2.5-bin/src")
#list(APPEND ALL_LIBRARIES ${arco_LIBS})
