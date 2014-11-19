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


// Blueberry
//#include <berryISelectionService.h>
//#include <berryIWorkbenchWindow.h>

// Qmitk
#include "QmitkEndoscopyCalibrationView.h"
//#include "QmitkChessboardSchemeWidget.h"
#include "org_mitk_gui_qt_endoscopycalibration_Activator.h"
//Microservices
#include "usServiceReference.h"
#include "usModuleContext.h"
#include "usServiceEvent.h"
#include "usServiceInterface.h"

// Qt
//#include <QMessageBox>
//#include <QTimer>
//#include <QFileDialog>
#include <QKeyEvent>
#include <QFileDialog>
#include <tinyxml.h>
// Open CV
/*
#include <cv.h>
#include <highgui.h>
*/
/*
#include "QmitkOpenCVVideoControls.h"
#include <QmitkVideoBackground.h>
#include <QmitkStdMultiWidget.h>
#include <mitkOpenCVVideoSource.h>
#include <mitkRenderWindow.h>
*/

// MITK-IGT
//#include "mitkNDIPassiveTool.h"
/*
#include "mitkStandardFileLocations.h"
#include "mitkSerialCommunication.h"
#include "mitkNavigationData.h"


#include "mitkNavigationDataRecorder.h"
*/
/*
#include <iostream>
#include <fstream>
#include <sstream>
*/
const std::string QmitkEndoscopyCalibrationView::VIEW_ID = "org.mitk.views.endoscopycalibration";

QmitkEndoscopyCalibrationView::QmitkEndoscopyCalibrationView():
QmitkAbstractView(),m_StopButtonClicked(false),m_Capture(NULL),m_ActualIndexOfChessboardPoint(-1),m_Source(NULL),m_Tracker(NULL),m_CalibrationStarted(false),m_LoggedPoint(0,0),m_SceneNumber(0)
{
  this->m_ChessboardPoints = mitk::ChessboardPoints::New();
  m_CountScreenshots = 0;

  m_PositionAndOrientationMatrixOfRigidBody = cv::Mat::zeros(4,4,CV_64F);
  m_EndoscopyController = mitk::EndoscopyController::New();
}

bool QmitkEndoscopyCalibrationView::eventFilter(QObject * /*obj*/, QEvent *e)
{
  // Key event to use pedal for login points
  if(e->type() == QEvent::KeyPress){
    QKeyEvent *k = (QKeyEvent *)e;
    if(k->key() == Qt::Key_F9){
      MITK_INFO << "pedal";

      OnLoginChessboardPointButtonClicked();

      return true;
    }
  }
  return false;
}



QmitkEndoscopyCalibrationView::~QmitkEndoscopyCalibrationView()
{
  m_Capture = NULL;

}

void QmitkEndoscopyCalibrationView::OnLoadIntrinsicsClicked()
{
  cv::Mat focalLength = cv::Mat::zeros(2,1,CV_64F);
  cv::Mat principalPoint = cv::Mat::zeros(2,1,CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(4,1,CV_64F);


  //xml reader reads xml file with intrinsics
  QFileDialog dialog;
  dialog.setNameFilter(tr("XML files (*.xml)"));

  QString fileName = dialog.getOpenFileName();
  if(fileName.isEmpty())
  {
    return;
  }
  TiXmlDocument doc(fileName.toStdString());
  bool loadOkay = doc.LoadFile();
  if(!loadOkay)
  {
    MITK_INFO << "Could not open file "+fileName.toStdString()+" for reading.";
  }
  TiXmlHandle hDoc(&doc);
  TiXmlElement* root = hDoc.FirstChildElement("CalibrationProject").ToElement();

  if(root)
  {
    TiXmlElement* results;
    results = root->FirstChildElement("results");

    if(results)
    {
      TiXmlElement* focusLengthX = results->FirstChildElement("focus_lenX");
      if(focusLengthX)
      {
        focalLength.at<double>(0,0) = GetIntrinsicsOutofXMLFile(focusLengthX);
      }
      TiXmlElement* focusLengthY = results->FirstChildElement("focus_lenY");
      if(focusLengthY)
      {
       focalLength.at<double>(1,0) = GetIntrinsicsOutofXMLFile(focusLengthY);
      }
      TiXmlElement* PrincipalX = results->FirstChildElement("PrincipalX");
      if(PrincipalX)
      {
        principalPoint.at<double>(0,0) = GetIntrinsicsOutofXMLFile(PrincipalX);
      }
      TiXmlElement* PrincipalY = results->FirstChildElement("PrincipalY");
      if(PrincipalY)
      {
        principalPoint.at<double>(1,0) = GetIntrinsicsOutofXMLFile(PrincipalY);
      }
      TiXmlElement* dist1 = results->FirstChildElement("Dist1");
      if(dist1)
      {
        distCoeffs.at<double>(0,0) = GetIntrinsicsOutofXMLFile(dist1);
      }
      TiXmlElement* dist2 = results->FirstChildElement("Dist2");
      if(dist2)
      {
        distCoeffs.at<double>(1,0) = GetIntrinsicsOutofXMLFile(dist2);
      }
      TiXmlElement* dist3 = results->FirstChildElement("Dist3");
      if(dist3)
      {
        distCoeffs.at<double>(2,0) = GetIntrinsicsOutofXMLFile(dist3);
      }
      TiXmlElement* dist4 = results->FirstChildElement("Dist4");
      if(dist4)
      {
        distCoeffs.at<double>(3,0) = GetIntrinsicsOutofXMLFile(dist4);
      }

    }

  }

  //save xml file in
  this->m_EndoscopyController->SaveCameraIntrinsics(focalLength,principalPoint,distCoeffs);
}

void QmitkEndoscopyCalibrationView::CreateQtPartControl( QWidget *parent )
{

  // create GUI widget
  m_Controls = Ui::QmitkEndoscopyCalibrationViewControls();
  m_Controls.setupUi(parent);
  this->CreateConnections();
  this->m_Controls.m_MeasurementSchemeChessboard->InitializeMeasurementPoints(m_ChessboardPoints);
  GetDataStorage()->Add(m_ChessboardPoints->GetPointSetNode());
  //install keyboard events
  QWidgetList widgetList = QApplication::allWidgets();
  for(int i = 0; i < widgetList.size(); i++)
  {
    widgetList.at(i)->installEventFilter(this);
  }

  m_Controls.m_ToolStatusWidget->SetTextAlignment(Qt::AlignLeft);
  m_Controls.m_ToolStatusWidget->SetShowPositions(true);
}
double QmitkEndoscopyCalibrationView::GetIntrinsicsOutofXMLFile(TiXmlElement* element)
{

 if(element)
      {
        std::istringstream i(element->GetText());
        double x;
        double result;
        if (!(i >> x))
          result = 0;
        result = x;
        return result;
      }

}

void QmitkEndoscopyCalibrationView::CreateConnections()
{
  connect( (QObject*)(m_Controls.m_StartButton), SIGNAL(clicked()),(QObject*) this, SLOT(OnStartEndoscopyButtonClicked()));
  connect( (QObject*)(m_Controls.m_StopButton), SIGNAL(clicked()),(QObject*) this, SLOT(OnStopEndoscopyButtonClicked()));
  connect( (QObject*)(m_Controls.m_TakeScreenshotButton), SIGNAL(clicked()),(QObject*) this, SLOT(OnTakeScreenshotButtonClicked()));
  connect( (QObject*)(m_Controls.m_Calibrate), SIGNAL(clicked()),(QObject*) this, SLOT(OnSolvePnPButtonClicked()));
  connect( (QObject*)(m_Controls.m_LoginPointOnChessboardButton),SIGNAL(clicked()),(QObject*) this, SLOT(OnLoginChessboardPointButtonClicked()));
  connect( (QObject*)(m_Controls.m_MeasurementSchemeChessboard), SIGNAL(PointSelected(int)), (QObject*) this, SLOT(SelectedPointHasChanged(int)) );
  connect( (QObject*)(m_Controls.m_ShowPointInVideo), SIGNAL(clicked()),(QObject*) this,SLOT(OnShowPointInVideoClicked()));
  connect( (QObject*)(m_Controls.m_LoadIntrinsics), SIGNAL(clicked()),(QObject*) this, SLOT(OnLoadIntrinsicsClicked()));
}

void QmitkEndoscopyCalibrationView::SetFocus()
{

}



void QmitkEndoscopyCalibrationView::SelectedPointHasChanged(int i)
{
  m_ActualIndexOfChessboardPoint = i;


}
// saves the coordinates of the selected point on chessboard
void QmitkEndoscopyCalibrationView::OnLoginChessboardPointButtonClicked()
{
  MITK_INFO << "RIGID BODY: " << m_Source->GetOutput(0)->GetPosition();
  mitk::PointSet::PointType pos = m_Source->GetOutput(1)->GetPosition();
  //MITK_INFO << "Point3D" << pos;
  this->m_ChessboardPoints->GetMPointAtPosition(m_ActualIndexOfChessboardPoint)->SetPoint3D(pos);
  this->m_ChessboardPoints->GetMPointAtPosition(m_ActualIndexOfChessboardPoint)->SetIsPointLogged(true);
  this->m_ChessboardPoints->Update();
  this->m_ChessboardPoints->UpdatePointSet();

  //MITK_INFO << m_ActualIndexOfChessboardPoint;
  //MITK_INFO << m_Source->GetOutput(1)->GetPosition();
  for(int i = 0; i < m_ChessboardPoints->GetNumberOfPoints();i++)
  {
    MITK_INFO << m_ChessboardPoints->GetMPointAtPosition(i)->GetPoint3D();
  }
  bool allChessboardPointsAreLogged = true;
  for(int i = 0; i < m_ChessboardPoints->GetNumberOfPoints();i++)
  {
    MITK_INFO << m_ChessboardPoints->GetMPointAtPosition(i)->GetPoint3D();
    if(!(m_ChessboardPoints->GetMPointAtPosition(i)->GetIsPointLogged()))
    {
      allChessboardPointsAreLogged = false;
    }
  }
  if(allChessboardPointsAreLogged)
  {
    this->m_Controls.m_Calibrate->setEnabled(true);
  }
}
// method is calculating a matrix that represents the offset between the rigid body and
// the origin of the camera coordinate system
void QmitkEndoscopyCalibrationView::OnSolvePnPButtonClicked()
{
  //IMAGE POINTS
  cv::Mat img = cv::imread("D:/home/s.kolb/Screenshot0.jpg");

  cv::Mat corners = cv::Mat::zeros(9,6,CV_64F);

  int found = cv::findChessboardCorners(img,cv::Size( 9, 6 ),corners);
  cv::Point2f leftUpperCornerImagePoint = corners.at<cv::Point2f>(0,0);
  cv::Point2f rightUpperCornerImagePoint = corners.at<cv::Point2f>(8,0);
  //cv::Point2f middleLowerCorner = corners.at<cv::Point2f>(39,0);
  cv::Point2f leftLowerCornerImagePoint = corners.at<cv::Point2f>(45,0);
  cv::Point2f rightLowerCornerImagePoint = corners.at<cv::Point2f>(53,0);

  // imagePoints is a vector conasisting the outer four corner points of a chessboard
  // the points are in open cv coordinates and detected with the finChessboardCorners Method from open cv
  // all coordinates are two-dimensional

  // write the coordinates to a vector for "solvePnP" Method
  std::vector<cv::Point2f> imagePoints(4);
  imagePoints[0] = leftUpperCornerImagePoint;
  imagePoints[1] = rightUpperCornerImagePoint;
  imagePoints[2] = leftLowerCornerImagePoint;
  imagePoints[3] = rightLowerCornerImagePoint;
  //imagePoints[4] = middleLowerCorner;
  //MITK_INFO << leftUpperCorner << "---" << rightUpperCorner << "---" << leftLowerCorner << "---" << rightLowerCorner << "---" << middleLowerCorner;


  // OBJECT POINTS
  cv::Point3f leftUpperCornerObjectPoint = ConvertCvPoint3FToOpenCVCoordinateSystem(0);
  cv::Point3f rightUpperCornerObjectPoint = ConvertCvPoint3FToOpenCVCoordinateSystem(1);
  cv::Point3f leftLowerCornerObjectPoint = ConvertCvPoint3FToOpenCVCoordinateSystem(2);
  cv::Point3f rightLowerCornerObjectPoint = ConvertCvPoint3FToOpenCVCoordinateSystem(3);
  //cv::Point3f middleLowerCornerObjectPoint; = ConvertCvPoint3FToOpenCVCoordinateSystem(4);
  // write the cv::mat coordinates to cv::point3f, to save them in a vector for solvePnP Method
  MITK_INFO << leftUpperCornerObjectPoint << "---" << rightUpperCornerObjectPoint << "---" << leftLowerCornerObjectPoint << "---" << rightLowerCornerObjectPoint;

  // saving cv::point3f coordinates in a vector
  std::vector<cv::Point3f> objPoints(4);
  objPoints[0] = leftUpperCornerObjectPoint;
  objPoints[1] = rightUpperCornerObjectPoint;
  objPoints[2] = leftLowerCornerObjectPoint;
  objPoints[3] = rightLowerCornerObjectPoint;
  //objPoints[4] = middleLowerCornerObjectPoint;
  // solvePnP Method is calculation a rotation vector and a translation matrix
  // these vectors together can transform points from open cv coordinate system to the camera coordinate system
  cv::solvePnP(objPoints,imagePoints,this->m_EndoscopyController->GetCameraMatrix(),this->m_EndoscopyController->GetDistorsionCoeffs(),this->m_EndoscopyController->m_RotationVectorToCameraOrigin,m_EndoscopyController->m_TranslationVectorToCameraOrigin);
  //MITK_INFO << "rvec: " << this->m_EndoscopyController->m_RotationVectorToCameraOrigin;
  //MITK_INFO << "tvec: " << this->m_EndoscopyController->m_TranslationVectorToCameraOrigin;
  // use rodrigues method for generating a rotation Matrix out of the rotation vector
  cv::Rodrigues(m_EndoscopyController->m_RotationVectorToCameraOrigin,this->m_EndoscopyController->m_RotationMatrixToCameraOrigin);
  //MITK_INFO << "Rodrigues Matrix: " << m_RotationMatrixToCameraOrigin;


  m_PositionAndOrientationMatrixOfRigidBody = GetActualRigidBodyPositionAndOrientation();
  //MITK_INFO << "m_PositionAndOrientationMatrixOfRigidBody: " << m_PositionAndOrientationMatrixOfRigidBody;
  cv::Mat homogeneousTransformationOfCameraOrigin = m_EndoscopyController->CreateHomogeneousMatrix(this->m_EndoscopyController->m_RotationMatrixToCameraOrigin,this->m_EndoscopyController->m_TranslationVectorToCameraOrigin);
  this->m_EndoscopyController->m_DistanceRigidBodyToCameraOrigin = (this->m_EndoscopyController->CalculateDistanceFromARigidBodyToCameraOrigin(m_PositionAndOrientationMatrixOfRigidBody,homogeneousTransformationOfCameraOrigin));

  m_CalibrationStarted = true;

  
  this->m_Controls.m_ShowPointInVideo->setEnabled(true);
}

void QmitkEndoscopyCalibrationView::OnShowPointInVideoClicked()
{
  //cv::Mat actualPointForMultiplication = cv::Mat::zeros(4,1,CV_64F);
  MITK_INFO << "m_RotationVectorToCameraOrigin before recalc: " << m_EndoscopyController->m_RotationVectorToCameraOrigin;
  MITK_INFO << "m_TranslationVectorToCameraOrigin before recalc: " << m_EndoscopyController->m_TranslationVectorToCameraOrigin;
  this->m_EndoscopyController->RecalculateExtrinsicsTranslationAndRotationVector(this->GetActualRigidBodyPositionAndOrientation());
  cv::Point3f pointCoordinates;
  cv::Mat actualPositionOfTrackingTool = GetActualTrackingToolPosition();
  std::vector<cv::Point3f> objPoints(1);
  pointCoordinates.x = actualPositionOfTrackingTool.at<double>(0,0);
  pointCoordinates.y = actualPositionOfTrackingTool.at<double>(1,0);
  pointCoordinates.z = actualPositionOfTrackingTool.at<double>(2,0);
  objPoints[0] = pointCoordinates;

  std::vector<cv::Point2f> projectedPoints;

  cv::projectPoints(objPoints,m_EndoscopyController->m_RotationVectorToCameraOrigin,m_EndoscopyController->m_TranslationVectorToCameraOrigin,this->m_EndoscopyController->GetCameraMatrix(),this->m_EndoscopyController->GetDistorsionCoeffs(),projectedPoints);

  m_LoggedPoint = projectedPoints[0];
  MITK_INFO << "loggedPoint" << m_LoggedPoint;

}

// takes a screenshot for calculating four chessboard points
void QmitkEndoscopyCalibrationView::OnTakeScreenshotButtonClicked()
{

  std::stringstream convert;
  convert << this->m_CountScreenshots;
  cv::Mat frame = cv::Mat::zeros(720, 1280, CV_8UC3);
  m_Capture >> frame;
  cv::Mat temp = frame.clone();
  cv::undistort(temp,frame,this->m_EndoscopyController->GetCameraMatrix(),this->m_EndoscopyController->GetDistorsionCoeffs());
  std::string path;
  path = "D:/home/s.kolb/Screenshot";
  std::string temp_Counter = convert.str();
  path.append(temp_Counter);
  path.append(".jpg");
  imwrite(path, frame);
  m_CountScreenshots++;
  cv::namedWindow("TestScreenshot",1);


  cv::Mat img = cv::imread("D:/home/s.kolb/Screenshot0.jpg");

  cv::Mat corners = cv::Mat::zeros(9,6,CV_64F);

  int found = cv::findChessboardCorners(img,cv::Size( 9, 6 ),corners);
  cv::drawChessboardCorners(img,cv::Size(9,6),corners,true);
  imshow("TestScreenshot", img);
  this->m_Controls.m_LoginPointOnChessboardButton->setEnabled(true);
}

void QmitkEndoscopyCalibrationView::OnStartEndoscopyButtonClicked()
{
  us::ServiceReferenceU service;

  us::ModuleContext* context = us::GetModuleContext();
  std::vector<us::ServiceReferenceU> ref = context->GetServiceReferences("org.mitk.services.NavigationDataSource");
  for(std::vector<us::ServiceReferenceU>::iterator it = ref.begin(); it != ref.end(); ++it)
  {
    m_Source = dynamic_cast<mitk::TrackingDeviceSource*>(context->GetService(us::ServiceReference<mitk::NavigationDataSource>((*it))));
  }


 //ui->trackingDevicesServiceList->Initialize<mitk::NavigationDataSource>(mitk::NavigationDataSource::US_PROPKEY_DEVICENAME);

  this->m_Controls.m_StopButton->setEnabled(true);
  this->m_Controls.m_TakeScreenshotButton->setEnabled(true);
  cv::Mat actualPositionOfTrackingTool = cv::Mat::zeros(3,1,CV_64F);
  cv::Mat actualPositionOfRigidBody = cv::Mat::zeros(3,1,CV_64F);

  cv::Mat resultVector = cv::Mat::zeros(3,1,CV_64F);
  cv::Mat finalResult;
  cv::Mat finalHomogeneousResult = cv::Mat::zeros(4,4,CV_64F);;
  //------------------------- IGT PART

  //m_Source = mitk::TrackingDeviceSource::New();
  /*m_Tracker = mitk::NDITrackingDevice::New();

  m_Tracker->SetPortNumber(mitk::SerialCommunication::COM9);
  m_Tracker->SetBaudRate(mitk::SerialCommunication::BaudRate115200);

  m_Tracker->AddTool("CameraTrackingTool","E:/MITK_SEM/MITK/Plugins/org.mitk.gui.qt.endoscopycalibration/resources/8700449.rom");
  m_Tracker->AddTool("Tracking Tool", "E:/MITK_SEM/MITK/Plugins/org.mitk.gui.qt.endoscopycalibration/resources/NDI_Probe_Y_Large.rom");


  m_Source->SetTrackingDevice(m_Tracker);
  m_Source->Connect();*/


  m_Source->StartTracking();

  for(unsigned int i=0; i<m_Source->GetNumberOfOutputs(); i++)
  {
    m_Controls.m_ToolStatusWidget->AddNavigationData(m_Source->GetOutput(i));
  }
  this->m_Controls.m_ToolStatusWidget->ShowStatusLabels();

  //-------------------------- OPEN CV PART


  m_Capture = cv::VideoCapture(0);
  if(!m_Capture.isOpened())  // check if we succeeded
    MITK_INFO << " cannot open camera";
  cv::VideoWriter writer;

  m_Capture.set( CV_CAP_PROP_FRAME_WIDTH, 1280 );
  m_Capture.set( CV_CAP_PROP_FRAME_HEIGHT, 720 );
  m_Capture.set( CV_CAP_PROP_FPS , 25 );
  double fps = m_Capture.get(CV_CAP_PROP_FPS);
  writer.open("E:\\capture.avi", CV_FOURCC( 'M', 'J', 'P', 'G' ), fps, cv::Size( 1280, 720 ), true );
  if(!writer.isOpened())  // check if we succeeded
    MITK_INFO << "cannot open writer";


  cv::Mat frame = cv::Mat::zeros(720, 1280, CV_8UC3);


  cv::namedWindow("EndoscopyCalibration",1);
  //MITK_INFO << "Orientation of Rigid Body: " << ;
  for(;;)
  {
    //MITK_INFO << m_Source->GetOutput(0)->GetPosition();
    if(m_CalibrationStarted)
    {
      //TODO: project points on the fly
    }
    m_Capture >> frame; // get a new frame from camera
    m_Source->Update();
    m_Controls.m_ToolStatusWidget->Refresh();
    //input = source->GetOutput();
    cv::Mat temp = frame.clone();

    cv::undistort(temp,frame,this->m_EndoscopyController->GetCameraMatrix(),this->m_EndoscopyController->GetDistorsionCoeffs());
    //MITK_INFO << input->GetOrientation();
    //cv::line(frame,cv::Point(0,0),cv::Point(128000,72000),cv::Scalar(0,0,200),1,3,4);
    //MITK_INFO << "m_LoggedPoint: " << m_LoggedPoint;

    cv::circle(frame,m_LoggedPoint,5,cv::Scalar(0,0,200),1,3,4);
    cv::Mat imgPoints = cv::Mat::zeros(2,1,CV_64F);

    writer.write( frame );
    imshow("EndoscopyCalibration", frame);

    if(cv::waitKey(1) >= 0) break;
    if(m_StopButtonClicked)
    {
      MITK_INFO << "stop clicked";
      cvDestroyWindow("EndoscopyCalibration");
      break;
    }
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  m_StopButtonClicked = false;

}


cv::Point3f QmitkEndoscopyCalibrationView::ConvertCvPoint3FToOpenCVCoordinateSystem(int positionInArrayRelatedToScheme)
{

   return this->m_EndoscopyController->ConvertMeasurementPointToOpenCvCoordinateSystem(this->m_ChessboardPoints->GetMPointAtPosition(positionInArrayRelatedToScheme)->GetPoint3D());
}


cv::Mat QmitkEndoscopyCalibrationView::GetActualRigidBodyPositionAndOrientation()
{
  cv::Mat rotationMatrixOfRigidBody = this->m_EndoscopyController->ConvertFromNavigationDataToCVMat(m_Source->GetOutput(0)->GetRotationMatrix());
  // transform the rotation Matrix from the polaris coordinate system to open cv coordinate system
  rotationMatrixOfRigidBody = this->m_EndoscopyController->GetRotationFromPolarisToOpenCVCoordinateSystem() * rotationMatrixOfRigidBody;
  cv::Mat translationMatrixOfRigidBody = cv::Mat::zeros(3,1,CV_64F);
  translationMatrixOfRigidBody.at<double>(0,0) = m_Source->GetOutput()->GetPosition()[0];
  translationMatrixOfRigidBody.at<double>(1,0) = m_Source->GetOutput()->GetPosition()[1];
  translationMatrixOfRigidBody.at<double>(2,0) = m_Source->GetOutput()->GetPosition()[2];
  translationMatrixOfRigidBody = this->m_EndoscopyController->GetRotationFromPolarisToOpenCVCoordinateSystem() * translationMatrixOfRigidBody;
  return this->m_EndoscopyController->CreateHomogeneousMatrix(rotationMatrixOfRigidBody,translationMatrixOfRigidBody);


}
//Todo: return parameter to cv::Point3f and use EndoscopyController::ConvertMeasurementPointToOpenCvCoordinateSystem(mitk::Point3D point)
cv::Mat QmitkEndoscopyCalibrationView::GetActualTrackingToolPosition()
{
  cv::Mat translationMatrixOfTrackingTool = cv::Mat::zeros(3,1,CV_64F);
  translationMatrixOfTrackingTool.at<double>(0,0) = m_Source->GetOutput(1)->GetPosition()[0];
  translationMatrixOfTrackingTool.at<double>(1,0) = m_Source->GetOutput(1)->GetPosition()[1];
  translationMatrixOfTrackingTool.at<double>(2,0) = m_Source->GetOutput(1)->GetPosition()[2];
  translationMatrixOfTrackingTool = this->m_EndoscopyController->GetRotationFromPolarisToOpenCVCoordinateSystem() * translationMatrixOfTrackingTool;
  return translationMatrixOfTrackingTool;


}

void QmitkEndoscopyCalibrationView::OnStopEndoscopyButtonClicked()
{
  this->m_Source->StopTracking();
  this->m_Source->Disconnect();
  this->m_StopButtonClicked = true;
  this->m_Controls.m_StartButton->setEnabled(true);
  this->m_Controls.m_StopButton->setEnabled(false);
  this->m_Controls.m_Calibrate->setEnabled(false);
  this->m_Controls.m_LoginPointOnChessboardButton->setEnabled(false);
  this->m_Controls.m_ShowPointInVideo->setEnabled(false);

}
