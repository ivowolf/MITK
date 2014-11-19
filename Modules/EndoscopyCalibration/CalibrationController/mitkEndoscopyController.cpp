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


#include "mitkEndoscopyController.h"
#include <cv.h>
#include <highgui.h>

#include "QmitkOpenCVVideoControls.h"
#include <QmitkVideoBackground.h>
#include <QmitkStdMultiWidget.h>
#include <mitkOpenCVVideoSource.h>
#include <mitkRenderWindow.h>


#include "mitkNDIPassiveTool.h"

#include "mitkStandardFileLocations.h"
#include "mitkSerialCommunication.h"
#include "mitkNavigationData.h"

#include "mitkNavigationDataToPointSetFilter.h"
#include "mitkNavigationDataRecorder.h"
#include "mitkNavigationDataPlayer.h"
#include "mitkNavigationDataObjectVisualizationFilter.h"

#include <iostream>
#include <fstream>
#include <sstream>

namespace mitk{

  EndoscopyController::EndoscopyController():
  m_TranslationVectorToCameraOrigin(3,1,CV_64F),m_RotationVectorToCameraOrigin(3,1,CV_64F),m_RotationMatrixToCameraOrigin(3,3,CV_64F)
  {
  // Calculate rotation matrix for transform from Polaris coordinate system to open cv coordinate system
  // rotate 90 degree around the x-axis
  cv::Mat rotationX = (cv::Mat_<double>(3, 3) << 1,0,0,0, -1, 0, 0 , 0, -1);

  //rotate 180 degree around the z-axis
  cv::Mat rotationZ = (cv::Mat_<double>(3, 3) << 0,-1,0,1, 0, 0, 0 , 0, 1);
  // matrix multiplication for calculating a rotation matrix to convert data from polaris coordinate system to open cv coordinate system
  // all calculatings will be in open cv coordinate system
  m_RotationFromPolarisToOpenCVCoordinateSystem = cv::Mat::zeros(3,3,cv::DataType<double>::type);
  m_RotationFromPolarisToOpenCVCoordinateSystem = rotationZ * rotationX;
  m_RotationFromPolarisToOpenCVCoordinateSystem = cv::Mat::eye(3,3,CV_64F);
  m_DistanceRigidBodyToCameraOrigin = cv::Mat::zeros(4,4,CV_64F);
  MITK_INFO << m_RotationFromPolarisToOpenCVCoordinateSystem;

  cv::Mat tempCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  m_CameraMatrix = cv::getDefaultNewCameraMatrix(tempCameraMatrix);
  this->m_DistorsionCoeffs = cv::Mat::zeros(4, 1, CV_64F);

  }

  EndoscopyController::~EndoscopyController()
  {

  }

void EndoscopyController::SaveCameraIntrinsics(cv::Mat focalLength, cv::Mat principalPoint, cv::Mat distCoeffs)
  {

    // Camera matrix consists focal length and principal point
    // 0,0 is the X value of the focal length
    m_CameraMatrix.at<double>(0,0) = focalLength.at<double>(0,0);
    // 1,1 is the y value of the focal length
    m_CameraMatrix.at<double>(1,1) = focalLength.at<double>(1,0);
    // 0,2 is the x value of the principal point
    m_CameraMatrix.at<double>(0,2) = principalPoint.at<double>(0,0);
    // 1,2 is the y value of the principal point
    m_CameraMatrix.at<double>(1,2) = principalPoint.at<double>(1,0);
    // 2,2 must be 1 for open cv
    m_CameraMatrix.at<double>(2,2) = 1;
    // distorsion coefficients for undistort camera frames
    m_DistorsionCoeffs.at<double>(0,0) = distCoeffs.at<double>(0,0);
    m_DistorsionCoeffs.at<double>(1,0) = distCoeffs.at<double>(1,0);
    m_DistorsionCoeffs.at<double>(2,0) = distCoeffs.at<double>(2,0);
    m_DistorsionCoeffs.at<double>(3,0) = distCoeffs.at<double>(3,0);

    MITK_INFO << "m_CameraMatrix: " << m_CameraMatrix;
    MITK_INFO << "m_DistorsionCoeffs: " << m_DistorsionCoeffs;
  }

  cv::Mat EndoscopyController::ConvertFromNavigationDataToCVMat(mitk::Matrix3D data)
  {
    cv::Mat rotationMatrix = cv::Mat::zeros(3,3,CV_64F);
    vnl_matrix_fixed<double,3,3> mat = data.GetVnlMatrix();

    rotationMatrix.at<double>(0,0) = mat.get(0,0);
    rotationMatrix.at<double>(0,1) = mat.get(0,1);
    rotationMatrix.at<double>(0,2) = mat.get(0,2);
    rotationMatrix.at<double>(1,0) = mat.get(1,0);
    rotationMatrix.at<double>(1,1) = mat.get(1,1);
    rotationMatrix.at<double>(1,2) = mat.get(1,2);
    rotationMatrix.at<double>(2,0) = mat.get(2,0);
    rotationMatrix.at<double>(2,1) = mat.get(2,1);
    rotationMatrix.at<double>(2,2) = mat.get(2,2);
    return rotationMatrix;

  }



  cv::Mat EndoscopyController::CreateHomogeneousMatrix(cv::Mat rotation,cv::Mat translation)
  {

    cv::Mat homogeneouseMatrix = cv::Mat::zeros(4,4,CV_64F);
    //fill homogeneous matrix with rotation matrix
    homogeneouseMatrix.at<double>(0,0) = rotation.at<double>(0,0);
    homogeneouseMatrix.at<double>(0,1) = rotation.at<double>(0,1);
    homogeneouseMatrix.at<double>(0,2) = rotation.at<double>(0,2);
    homogeneouseMatrix.at<double>(1,0) = rotation.at<double>(1,0);
    homogeneouseMatrix.at<double>(1,1) = rotation.at<double>(1,1);
    homogeneouseMatrix.at<double>(1,2) = rotation.at<double>(1,2);
    homogeneouseMatrix.at<double>(2,0) = rotation.at<double>(2,0);
    homogeneouseMatrix.at<double>(2,1) = rotation.at<double>(2,1);
    homogeneouseMatrix.at<double>(2,2) = rotation.at<double>(2,2);

    //fill homogeneous matrix with translation vector
    homogeneouseMatrix.at<double>(0,3) = translation.at<double>(0,0);
    homogeneouseMatrix.at<double>(1,3) = translation.at<double>(1,0);
    homogeneouseMatrix.at<double>(2,3) = translation.at<double>(2,0);

    //fill last component with 1
    homogeneouseMatrix.at<double>(3,3) = 1;

    return homogeneouseMatrix;
  }

  cv::Mat EndoscopyController::CalculateDistanceFromARigidBodyToCameraOrigin(cv::Mat rigidBodyOrientationAndTranslation,cv::Mat cameraOrigin)
  {

    cv::Mat inverseHomogeneousTransformationOfRigidBody = rigidBodyOrientationAndTranslation;
    cv::Mat distanceFromARigidBodyToCameraOrigin;
    distanceFromARigidBodyToCameraOrigin = cameraOrigin * inverseHomogeneousTransformationOfRigidBody;
    //MITK_INFO << "DistanceFromARigidBodyToCameraOriginMethod: " << distanceFromARigidBodyToCameraOrigin;
    return cameraOrigin * inverseHomogeneousTransformationOfRigidBody;


  }

  cv::Mat EndoscopyController::RecalculatingCameraOrigin(cv::Mat rigidBodyOrientationAndTranslation, cv::Mat distanceFromRigidBodyToCameraOrigin)
  {
    MITK_INFO << "rigidBodyOrientationAndTranslation: " << rigidBodyOrientationAndTranslation;
    MITK_INFO << "distanceFromRigidBodyToCameraOrigin: " << distanceFromRigidBodyToCameraOrigin;
    cv::Mat newCameraOrigin;
    newCameraOrigin = distanceFromRigidBodyToCameraOrigin * rigidBodyOrientationAndTranslation.inv();
    MITK_INFO << "newCameraOriginMethod:" << newCameraOrigin;
    return distanceFromRigidBodyToCameraOrigin * rigidBodyOrientationAndTranslation.inv();


  }

  cv::Point3f EndoscopyController::ConvertMeasurementPointToOpenCvCoordinateSystem(mitk::Point3D point)
  {

    cv::Point3f tempPoint;
    cv::Mat matrixForMultiplication = cv::Mat::zeros(3, 1,CV_64F);
    tempPoint.x = point[0];
    matrixForMultiplication.at<double>(0,0) = tempPoint.x;
    tempPoint.y = point[1];
    matrixForMultiplication.at<double>(1,0) = tempPoint.y;
    tempPoint.z = point[2];
    matrixForMultiplication.at<double>(2,0) = tempPoint.z;
    // CONVERT FROM POLARIS TO OPENCV COORDINATE SYSTEM
    matrixForMultiplication = m_RotationFromPolarisToOpenCVCoordinateSystem * matrixForMultiplication;
    tempPoint.x = matrixForMultiplication.at<double>(0,0);
    tempPoint.y = matrixForMultiplication.at<double>(1,0);
    tempPoint.z = matrixForMultiplication.at<double>(2,0);

    return tempPoint;

  }

  void EndoscopyController::RecalculateExtrinsicsTranslationAndRotationVector(cv::Mat rigidBodyTranslationAndOrientation)
  {

    cv::Mat newCameraOrigin = this->RecalculatingCameraOrigin(rigidBodyTranslationAndOrientation,m_DistanceRigidBodyToCameraOrigin);

    this->m_RotationMatrixToCameraOrigin.at<double>(0,0) = newCameraOrigin.at<double>(0,0);
    this->m_RotationMatrixToCameraOrigin.at<double>(0,1) = newCameraOrigin.at<double>(0,1);
    this->m_RotationMatrixToCameraOrigin.at<double>(0,2) = newCameraOrigin.at<double>(0,2);
    this->m_RotationMatrixToCameraOrigin.at<double>(1,0) = newCameraOrigin.at<double>(1,0);
    this->m_RotationMatrixToCameraOrigin.at<double>(1,1) = newCameraOrigin.at<double>(1,1);
    this->m_RotationMatrixToCameraOrigin.at<double>(1,2) = newCameraOrigin.at<double>(1,2);
    this->m_RotationMatrixToCameraOrigin.at<double>(2,0) = newCameraOrigin.at<double>(2,0);
    this->m_RotationMatrixToCameraOrigin.at<double>(2,1) = newCameraOrigin.at<double>(2,1);
    this->m_RotationMatrixToCameraOrigin.at<double>(2,2) = newCameraOrigin.at<double>(2,2);
    cv::Rodrigues(m_RotationMatrixToCameraOrigin,this->m_RotationVectorToCameraOrigin);
    //MITK_INFO << "m_RotationVectorToCameraOrigin recalced" << m_RotationVectorToCameraOrigin;
    this->m_TranslationVectorToCameraOrigin.at<double>(0,0) = newCameraOrigin.at<double>(0,3);
    this->m_TranslationVectorToCameraOrigin.at<double>(1,0) = newCameraOrigin.at<double>(1,3);
    this->m_TranslationVectorToCameraOrigin.at<double>(2,0) = newCameraOrigin.at<double>(2,3);
    //MITK_INFO << "m_TranslationVectorToCameraOrigin recalced" << m_TranslationVectorToCameraOrigin;


  }




} // end namespace mitk
