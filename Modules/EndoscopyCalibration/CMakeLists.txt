
MITK_CREATE_MODULE(
  DEPENDS MitkOpenCVVideoSupport MitkIGT MitkOpenCVVideoSupportUI MitkIGTUI
  PACKAGE_DEPENDS VTK OpenCV
  INCLUDE_DIRS Datastructure CalibrationController
  EXPORT_DEFINE MITK_ENDOSCOPYCALIBRATION_EXPORT
  
)
add_subdirectory(Testing)
