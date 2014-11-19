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


#ifndef EndoscopyCalibrationView_h
#define EndoscopyCalibrationView_h

//#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_QmitkEndoscopyCalibrationViewControls.h"
#include <mitkChessboardPoints.h>
//#include <mitkMeasurementsPoints.h>


#include <tinyxml.h>
//#include <berryQtViewPart.h>
//#include <berryISelectionService.h>
//#include <berryIWorkbenchWindow.h>

#include <mitkEndoscopyController.h>

//#include <QAction>


//#include <cv.h>
/*
#include "QmitkOpenCVVideoControls.h"
#include <mitkOpenCVVideoSource.h>
*/
/*
#include "mitkNDITrackingDevice.h"
#include "mitkTrackingDeviceSource.h"
*/
/*!
  \brief EndoscopyCalibration

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkFunctionality
  \ingroup ${plugin_target}_internal
*/
class QmitkEndoscopyCalibrationView : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

  public:

    static const std::string VIEW_ID;
    QmitkEndoscopyCalibrationView();
    ~QmitkEndoscopyCalibrationView();
    virtual void CreateQtPartControl(QWidget *parent);
    virtual void CreateConnections();


signals:
    protected slots:
      virtual void SetFocus();
      void OnLoginChessboardPointButtonClicked();
      void OnStartEndoscopyButtonClicked();
      void OnStopEndoscopyButtonClicked();
      void OnTakeScreenshotButtonClicked();
      void OnSolvePnPButtonClicked();
      void OnShowPointInVideoClicked();
      void SelectedPointHasChanged(int);
      void OnLoadIntrinsicsClicked();

  protected:



    /// \brief called by QmitkFunctionality when DataManager's selection has changed
    double GetIntrinsicsOutofXMLFile(TiXmlElement* element);
    Ui::QmitkEndoscopyCalibrationViewControls m_Controls;
    cv::Point3f ConvertCvPoint3FToOpenCVCoordinateSystem(int positionInArrayRelatedToScheme);
    cv::Mat GetActualRigidBodyPositionAndOrientation();
    cv::Mat GetActualTrackingToolPosition();
    bool eventFilter(QObject *obj, QEvent *event);
    bool m_StopButtonClicked;
    /// \brief called by QmitkFunctionality when DataManager's selection has changed
    cv::VideoCapture m_Capture;
    cv::VideoWriter m_Writer;

    int m_CountScreenshots;
    mitk::ChessboardPoints::Pointer m_ChessboardPoints;
    mitk::TrackingDeviceSource::Pointer m_Source;
    mitk::NDITrackingDevice::Pointer m_Tracker;
    int m_ActualIndexOfChessboardPoint;
    cv::Mat m_PositionAndOrientationMatrixOfRigidBody;

    // this is a 4x4 homogeneous matrix with rotation and translation
    cv::Mat m_HomogeneousTransformationOfCameraOrigin;
    bool m_CalibrationStarted;
    cv::Point m_LoggedPoint;
    int m_SceneNumber;
    std::ofstream m_Testfile;
private:
  mitk::EndoscopyController::Pointer m_EndoscopyController;

};

#endif // EndoscopyCalibrationView_h

