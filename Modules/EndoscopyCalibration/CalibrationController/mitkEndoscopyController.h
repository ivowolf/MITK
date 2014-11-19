/*=========================================================================

Program:   Medical Imaging & Interaction Toolkit
Language:  C++
Date:      $Date$
Version:   $Revision$

Copyright (c) German Cancer Research Center, Division of Medical and
Biological Informatics. All rights reserved.
See MITKCopyright.txt or http://www.mitk.org/copyright.html for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notices for more information.

=========================================================================*/


#ifndef MITK_EndoscopyController_H
#define MITK_EndoscopyController_H

#include <itkObject.h>
#include <MitkEndoscopyCalibrationExports.h>
#include <mitkCommon.h>
//#include <mitkChessboardPoints.h>

#include <cv.h>
#include "QmitkOpenCVVideoControls.h"
#include <mitkOpenCVVideoSource.h>
#include "mitkNDITrackingDevice.h"
#include "mitkTrackingDeviceSource.h"

/*!
\brief EndoscopyCalibration

\warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

*/

namespace mitk
{
  class MITK_ENDOSCOPYCALIBRATION_EXPORT EndoscopyController : public itk::Object
  {


  public:

    mitkClassMacro(EndoscopyController, itk::Object);
    itkFactorylessNewMacro(Self)
    itkCloneMacro(Self)
    cv::Mat ConvertFromNavigationDataToCVMat(mitk::Matrix3D);
    cv::Mat CreateHomogeneousMatrix(cv::Mat,cv::Mat);
    // Second parameter is the position of the point to connect it with the point in the scheme widget
    cv::Mat CalculateDistanceFromARigidBodyToCameraOrigin(cv::Mat rigidBodyOrientationAndTranslation,cv::Mat cameraOrigin);
    //recalculates the origin of the camera
    cv::Mat RecalculatingCameraOrigin(cv::Mat rigidBodyOrientationAndTranslation,cv::Mat distanceFromRigidBodyToCameraOrigin);
    cv::Point3f ConvertMeasurementPointToOpenCvCoordinateSystem(mitk::Point3D);
    void RecalculateExtrinsicsTranslationAndRotationVector(cv::Mat rigidBodyTranslationAndOrientation);

    itkGetMacro(RotationFromPolarisToOpenCVCoordinateSystem, cv::Mat);
    itkGetMacro(CameraMatrix, cv::Mat);
    itkGetMacro(DistorsionCoeffs,cv::Mat);
    cv::Mat m_DistanceRigidBodyToCameraOrigin;
    cv::Mat m_TranslationVectorToCameraOrigin;
    cv::Mat m_RotationVectorToCameraOrigin;
    cv::Mat m_RotationMatrixToCameraOrigin;
    void SaveCameraIntrinsics(cv::Mat focalLength, cv::Mat principalPoint, cv::Mat distCoeffs);
  protected:
    EndoscopyController();
    ~EndoscopyController();
    cv::Mat m_RotationFromPolarisToOpenCVCoordinateSystem;
    cv::Mat m_CameraMatrix;
    cv::Mat m_DistorsionCoeffs;


  private:

  };
}
#endif // __EndoscopyController_