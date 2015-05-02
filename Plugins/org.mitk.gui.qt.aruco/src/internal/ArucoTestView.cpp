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
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "ArucoTestView.h"

// Qt
#include <QMessageBox>

#include "aruco.h"

#include <mitkNavigationTool.h>
#include <mitkDataNode.h>
#include <mitkImage.h>
#include <mitkSurface.h>
#include <mitkCone.h>
#include <mitkExtractSliceFilter.h>
#include <mitkIRenderingManager.h>

#include "cvdrawingutils.h"

using namespace cv;
using namespace aruco;

mitk::Point3D camPos;
mitk::Point3D focalPoint;
mitk::Point3D markerPos;

cv::Mat pTvec;
cv::Mat pRvec;

string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
cv::Mat TheInputImageCopy;
aruco::MarkerDetector MDetector;

cv::Mat ExtrinsicRotation;
cv::Mat ExtrinsicTranslation;

vtkSmartPointer<vtkMatrix4x4> ExtrinsicTransformation;

aruco::BoardDetector BDetector;

cv::VideoCapture TheVideoCapturer;
vector<aruco::Marker> TheMarkers;
aruco::CameraParameters TheCameraParameters;

BoardConfiguration BC;
CameraParameters CP;

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;

cv::Mat TheInputImage;
void cvTackBarEvents(int pos,void*);

const std::string ArucoTestView::VIEW_ID = "org.mitk.views.arucotestview";

ArucoTestView::ArucoTestView()
  : m_VideoSource(0), m_Timer(NULL), m_Slicing(false) 
{
  m_RefImage = mitk::Image::New();
  m_TrackingDeviceSource = mitk::TrackingDeviceSource::New();
  m_ArUcoTrackingDevice = mitk::ArUcoTrackingDevice::New();
  m_ToolStorage = mitk::NavigationToolStorage::New();
  m_SelectedImageNode = mitk::DataNode::New();
  m_SlicedImage = mitk::DataNode::New();
  m_Transformation = mitk::AffineTransform3D::New();
  m_SlicedImage->SetName("SlicedImage");
  m_SlicedImage->SetVisibility(true, mitk::BaseRenderer::GetInstance
                               ( mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget1")));
  m_SlicedImage->SetVisibility(true, mitk::BaseRenderer::GetInstance
                               ( mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget4")));

  ExtrinsicTransformation = vtkSmartPointer<vtkMatrix4x4>::New();
}

ArucoTestView::~ArucoTestView()
{
  if(m_VideoSource.IsNotNull() && m_VideoSource->IsCapturingEnabled())
  {
    m_VideoSource->StopCapturing();
  }
}

void ArucoTestView::SetFocus()
{
  m_Controls.buttonPerformImageProcessing->setFocus();
}

void ArucoTestView::CreateQtPartControl( QWidget *parent )
{
  m_Controls.setupUi( parent );
  connect( m_Controls.buttonSetupTracker, SIGNAL(clicked()), this, SLOT(SetupArUcoTracker()) );
  connect( m_Controls.buttonPerformImageProcessing, SIGNAL(clicked()), this, SLOT(DoImageProcessing()) );
  connect( m_Controls.buttonStart, SIGNAL(clicked()), this, SLOT(Start()) );
  connect( m_Controls.btnSlice, SIGNAL(clicked()), this, SLOT(GetSliceFromMarkerPosition()) );
  connect( m_Controls.boxSlicing, SIGNAL(toggled(bool)), this, SLOT(SetPermanentSlicing(bool)));
  connect( m_Controls.btnCalibrate, SIGNAL(clicked()), this, SLOT(CalibrateProbe()));
  connect( m_Controls.btnSetImage, SIGNAL(clicked()), this, SLOT(SetRefImage()));
  connect( m_Controls.btnTransform, SIGNAL(clicked()), this, SLOT(SetTransformation()) );
  connect( m_Controls.btnSliceSl, SIGNAL(clicked()), this, SLOT(TestSliceSelector()) );

  connect( m_Controls.btnTest, SIGNAL(clicked()), this, SLOT(CameraTest()) );
  connect( m_Controls.btnBugTest, SIGNAL(clicked()), this, SLOT(GeoBugTest()) );

  // retrieve old preferences
  m_VideoSource = mitk::OpenCVVideoSource::New();
  m_VideoBackground = new QmitkVideoBackground(m_VideoSource);
  m_VideoBackground->setParent(parent);

  m_VideoSource->SetVideoCameraInput(0);
  m_ArUcoTrackingDevice->SetVideoSource(m_VideoSource);
}

void ArucoTestView::GeoBugTest()
{
//  mitk::NavigationData::Pointer navData = m_TrackingDeviceSource->GetOutput();
//  mitk::Quaternion orientation = navData->GetOrientation();

//  mitk::Matrix3D rotaMat;
//  for(int i=0; i<3; i++)
//  {
//    for(int j=0; j<3; j++)
//    {
//      rotaMat[i][j] = orientation.rotation_matrix_transpose().transpose()[i][j];
//    }
//  }

  mitk::Matrix3D m_Rotation;
  m_Rotation.Fill(0);
  m_Rotation[0][0] = 1;
  m_Rotation[1][1] = 2;
  m_Rotation[0][2] = 1;
  m_Rotation[2][2] = 1;

  mitk::Point3D point; point[0]=2; point[1]=5; point[2]=8;

  mitk::Point3D final;
  for(int i=0;i<3;i++)
  {
      final[i] =  m_Rotation[i][0] * point [0] + m_Rotation[i][1] * point[1] + m_Rotation[i][2] * point[2];
  }

  MITK_WARN << "MAT: " << final;
}

void ArucoTestView::TestSliceSelector()
{
    QmitkRenderWindow* renderwindow = this->GetRenderWindowPart()->GetQmitkRenderWindow("axial");
    renderwindow->GetSliceNavigationController()->GetSlice()->SetPos(10);

    mitk::DataNode::Pointer sagittal = this->GetDataStorage()->GetNamedNode("stdmulti.widget2.plane");
    mitk::DataNode::Pointer coronal = this->GetDataStorage()->GetNamedNode("stdmulti.widget3.plane");
    sagittal->SetVisibility(false);
    coronal->SetVisibility(false);
}

void ArucoTestView::SetTransformation()
{
    mitk::Surface::Pointer surf = dynamic_cast<mitk::Surface*>(m_SelectedImageNode->GetData());
    if(surf.IsNotNull())
    {
        m_Transformation = surf->GetGeometry()->GetIndexToWorldTransform()->Clone();
        std::cout << "ist leer: " << m_Transformation.IsNull() << std::endl;
    }
}

#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <mitkPoint.h>
#include <mitkCuboid.h>

void ArucoTestView::CameraTest()
{
  //    mitk::Cone::Pointer cone = mitk::Cone::New();

  //    mitk::DataNode::Pointer pointNode = mitk::DataNode::New();
  //    pointNode->SetData(cone);
  //    pointNode->SetName("ConeNode");
  //    this->GetDataStorage()->Add(pointNode);

  mitk::BaseRenderer* renderer = mitk::BaseRenderer::GetInstance(mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget4"));
  vtkRenderer* vtkRenderer = renderer->GetVtkRenderer();
  vtkCamera* camera = vtkRenderer->GetActiveCamera();

  //The size of the renderwindows?!
//  vtkRenderer->GetSize(); /*or*/ int* width, height; vtkRenderer->GetTiledSize(width,height);
  renderer->GetSizeX(); renderer->GetSizeY();

  if(camera)
  {
//    camera->SetPosition(0, -6.69213, 5.5);
//    camera->SetPosition(0.179, -0.984, 0.004); //0.179 -0.984 0.004
//    camera->SetPosition(camPos[0],camPos[1],camPos[2]);
//    camera->SetFocalPoint(0, 0, 0);
//    camera->SetViewUp(0, 0, 1);

    camera->SetPosition(0, 0, 0);
    camera->SetFocalPoint(0, 0, 1); // hier mit - testen
    camera->SetViewUp(0, 1, 0); // hier auch + alle kombinationen
  }
  vtkRenderer->ResetCameraClippingRange();
}

void ArucoTestView::CamParamsTest()
{
  TheInputImage = m_VideoSource->GetImage();

  aruco::CameraParameters camParams;
  camParams.readFromXMLFile("/home/riecker/Development/src/Seminar/Plugins/org.mitk.gui.qt.aruco/out_camera_data.xml");

  cv::Mat intrinsics = camParams.CameraMatrix;
  double focalLengthY = intrinsics.at<float>(1,1); //focalLengthX is at 0,0 of matrix
  cv::Size sizeofCam = camParams.CamSize;

  mitk::BaseRenderer* renderer = mitk::BaseRenderer::GetInstance(mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget4"));
  vtkRenderer* vtkRenderer = renderer->GetVtkRenderer();
  vtkCamera* vtkCamera = vtkRenderer->GetActiveCamera();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(renderer->GetSizeY() != sizeofCam.height)
  {
      double factor = static_cast<double>(renderer->GetSizeY())/static_cast<double>(sizeofCam.height);
      focalLengthY *= factor;
  }

  double viewAngle = 2 * atan( (renderer->GetSizeY()/2) / focalLengthY ) * 180 /vnl_math::pi;

  vtkCamera->SetViewAngle(viewAngle);

  double px = 0.0, width = 0.0, py = 0.0, height = 0.0;

  if( renderer->GetSizeX() != sizeofCam.width || renderer->GetSizeY() != sizeofCam.height)
  {
      double factor = static_cast<double>(renderer->GetSizeY())/static_cast<double>(sizeofCam.height);
      px = factor * intrinsics.at<float>(0,2);
      width = renderer->GetSizeX();

      int expectedWindowSize = cvRound(factor * static_cast<double>(sizeofCam.width));
      if( expectedWindowSize != renderer->GetSizeX() )
      {
          int diffX = (renderer->GetSizeX() - expectedWindowSize) / 2;
          px = px + diffX;
      }

      py = factor * intrinsics.at<float>(1,2);
      height = renderer->GetSizeY(); //that cant be correct ?!
  }
  else
  {
      px = intrinsics.at<float>(0,2);
      width = renderer->GetSizeX();

      py = intrinsics.at<float>(1,2);
      height = renderer->GetSizeY();
  }

  double cx = width - px;
  double cy = py;

  double newCenterX = cx / ( ( width-1)/2 ) - 1 ;
  double newCenterY = cy / ( ( height-1)/2 ) - 1;

  vtkCamera->SetWindowCenter(newCenterX, newCenterY);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  cv::Size sizeofCam = camParams.CamSize; //maybe here the X x Y Resolution of Cam Image

  //here get the renderwindow size

  //Detection of markers in the image passed
  /*
  MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,200);

  for(int i=0; i<TheMarkers.size();i++){
    Marker marker = TheMarkers.at(i);
    if(marker.id == 900)
    {
      double pos[3];
      double orient[4];
      marker.OgreGetPoseParameters(pos,orient);


      pTvec = marker.Tvec;
      pRvec = marker.Rvec;

      //            cv::Point3f tmp = TheCameraParameters.getCameraLocation(marker.Rvec, marker.Tvec); //error by calling this method ...
      //            std::cout << "EYE : " << tmp.x << " " << tmp.y << " " << tmp.z << std::endl;

      cv::Mat m33(3,3,CV_32FC1);
      cv::Rodrigues(marker.Rvec, m33)  ;

      cv::Mat m44=cv::Mat::eye(4,4,CV_32FC1);
      for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
          m44.at<float>(i,j)=m33.at<float>(i,j);

      //now, add translation information
      for (int i=0;i<3;i++)
        m44.at<float>(i,3)=marker.Tvec.at<float>(0,i);
      //invert the matrix
      m44.inv();
      cv::Point3f tmp = cv::Point3f( m44.at<float>(0,0),m44.at<float>(0,1),m44.at<float>(0,2));

      std::cout << "EYE : " << tmp.x << " " << tmp.y << " " << tmp.z << std::endl;
      //0.179 -0.984 0.004
    }
  }
  */
}

void ArucoTestView::SetRefImage()
{
  this->m_RefImage = dynamic_cast<mitk::Image*>(m_SelectedImageNode->GetData());

  if(m_RefImage.IsNull())
    MITK_ERROR << "Cant set the reference Image!" << std::endl;
}

#include <mitkImageSliceSelector.h>

void ArucoTestView::SetPermanentSlicing(bool slicing)
{
  m_Slicing = slicing;
}

void ArucoTestView::SetImageGeo()
{
  mitk::BaseGeometry* geo;
  mitk::Image::Pointer image = dynamic_cast<mitk::Image*>(m_SelectedImageNode->GetData());
  geo = image->GetGeometry();
  if(image->IsEmpty() || !geo->IsValid())
  {
    MITK_ERROR << "Image or Geometry is empty ... " << std::endl;
    return;
  }
  m_ArUcoTrackingDevice->SetPointGeometry(geo);
}

void ArucoTestView::OnSelectionChanged( berry::IWorkbenchPart::Pointer /*source*/,
                                        const QList<mitk::DataNode::Pointer>& nodes )
{
  foreach( mitk::DataNode::Pointer node, nodes )
  {
    if( node.IsNotNull() && dynamic_cast<mitk::Image*>(node->GetData()) )
    {
      m_Controls.btnSlice->setEnabled( true );
      m_SelectedImageNode = node;
      return;
    }
  }

  m_Controls.btnSlice->setEnabled( false );
}

void ArucoTestView::SetupArUcoTracker()
{
  m_Controls.boardDetectionLabel->setStyleSheet("QLabel { background-color : red;}");
  m_TrackingDeviceSource->SetTrackingDevice(m_ArUcoTrackingDevice);
  m_TrackingDeviceSource->RegisterAsMicroservice();
  m_ToolStorage->RegisterAsMicroservice(m_TrackingDeviceSource->GetMicroserviceID());

//  this->GetDataStorage()->Add(m_SlicedImage);

  mitk::NavigationTool::Pointer navigationTool = mitk::NavigationTool::New();
  navigationTool->SetTrackingTool(m_ArUcoTrackingDevice->GetTool(0));
  std::stringstream toolname;
  toolname << "AutoDetectedTool " << 0;
  //navigationTool->SetSerialNumber(dynamic_cast<mitk::NDIPassiveTool*>(currentDevice->GetTool(i))->GetSerialNumber());
  navigationTool->SetIdentifier(toolname.str());
  navigationTool->SetTrackingDeviceType(mitk::NDIAurora);
  mitk::DataNode::Pointer newNode = mitk::DataNode::New();

  mitk::Cone::Pointer cone = mitk::Cone::New();                 //instantiate a new cone
  newNode->SetData(cone);
  newNode->SetName(toolname.str());

  navigationTool->SetDataNode(newNode);

  m_ToolStorage->AddTool(navigationTool);
  this->GetDataStorageReference()->GetDataStorage()->Add(newNode);

  //As we wish to visualize our tool we need to have a PolyData which shows us the movement of our tool.
  //Here we take a cone shaped PolyData. In MITK you have to add the PolyData as a node into the DataStorage
  //to show it inside of the rendering windows. After that you can change the properties of the cone
  //to manipulate rendering, e.g. the position and orientation as in our case.
  //  float scale[] = {10.0, 10.0, 10.0};
  mitk::Vector3D scale;
  scale.Fill(100);
  cone->GetGeometry()->SetSpacing(scale);                       //scale it a little that so we can see something
  mitk::DataNode::Pointer node = mitk::DataNode::New(); //generate a new node to store the cone into
  //the DataStorage.
  node->SetData(cone);                          //The data of that node is our cone.
  node->SetName("My tracked object");           //The node has additional properties like a name
  node->SetColor(0.0, 0.0, 1.0);                //or the color. Here we make it red.
  this->GetDataStorageReference()->GetDataStorage()->Add(node); //After adding the Node with the cone in it to the
  //DataStorage, MITK will show the cone in the
  //render windows.

  //For updating the render windows we use another filter of the MITK-IGT pipeline concept. The
  //NavigationDataObjectVisualizationFilter needs as input a NavigationData and a
  //PolyData. In our case the input is the source and the PolyData our cone.

  //First we create a new filter for the visualization update.
  m_Visualizer = mitk::NavigationDataObjectVisualizationFilter::New();
  m_Visualizer->SetInput(0, m_TrackingDeviceSource->GetOutput(0));        //Then we connect to the pipeline.
  m_Visualizer->SetRepresentationObject(0, cone);  //After that we have to assign the cone to the input

  //Now this simple pipeline is ready, so we can start the tracking. Here again: We do not call the
  //StartTracking method from the tracker object itself. Instead we call this method from our source.
  m_TrackingDeviceSource->Connect();
  m_TrackingDeviceSource->StartTracking();

  //Now every call of m_Visualizer->Update() will show us the cone at the position and orientation
  //given from the tracking device.
  //We use a QTimer object to call this Update() method in a fixed interval.
  if (m_Timer == NULL)
  {
    m_Timer = new QTimer(this);  //create a new timer
  }
  connect(m_Timer, SIGNAL(timeout()), this, SLOT(OnTimer())); //connect the timer to the method OnTimer()

  m_Timer->start(100);  //Every 100ms the method OnTimer() is called. -> 10fps
  //Now have look at the OnTimer() method.

  //set the other two planes invisible to show only the axial plane in 3D
  mitk::DataNode::Pointer sagittal = this->GetDataStorage()->GetNamedNode("stdmulti.widget2.plane");
  mitk::DataNode::Pointer coronal = this->GetDataStorage()->GetNamedNode("stdmulti.widget3.plane");
  sagittal->SetVisibility(false);
  coronal->SetVisibility(false);

  mitk::RenderingManager::GetInstance()->InitializeViewsByBoundingObjects(this->GetDataStorageReference()->GetDataStorage());
}
#include <mitkPlaneGeometry.h>
void ArucoTestView::OnTimer()
{
  //Here we call the Update() method from the Visualization Filter. Internally the filter checks if
  //new NavigationData is available. If we have a new NavigationData the cone position and orientation
  //will be adapted.
  m_Visualizer->Update();
  TheInputImage = m_VideoSource->GetImage();

  BC.readFromFile("/home/riecker/Development/src/Seminar/Plugins/org.mitk.gui.qt.aruco/TestData/chessboardinfo_pix.yml");
  CP.readFromXMLFile("/home/riecker/Development/src/Seminar/Plugins/org.mitk.gui.qt.aruco/TestData/out_camera_data.xml");

  BDetector.setParams(BC,CP,100);
  BDetector.detect(TheInputImage);
  Board b = BDetector.getDetectedBoard();

  if(!b.empty())
  {
    m_Controls.boardDetectionLabel->setStyleSheet("QLabel { background-color : green;}");
  }
  else
  {
      m_Controls.boardDetectionLabel->setStyleSheet("QLabel { background-color : red;}");
  }

  mitk::NavigationData::Pointer navData = m_TrackingDeviceSource->GetOutput();
  markerPos = navData->GetPosition();

  if(m_Slicing)
  {
//    this->GetSliceFromMarkerPosition();
    mitk::NavigationData::Pointer navData = m_TrackingDeviceSource->GetOutput();
    mitk::Point3D pos = navData->GetPosition();
    mitk::BaseGeometry::Pointer geo = m_RefImage->GetGeometry();
    mitk::Point3D indexPos;
    geo->WorldToIndex(pos,indexPos);

    unsigned int* dimensions = m_RefImage->GetDimensions();

    unsigned int slicePos = static_cast<unsigned int>(indexPos[2]);
    QmitkRenderWindow* renderwindow = this->GetRenderWindowPart()->GetQmitkRenderWindow("axial");
    renderwindow->GetSliceNavigationController()->GetSlice()->SetPos(dimensions[2] - slicePos);

    unsigned int realposition = renderwindow->GetSliceNavigationController()->GetSlice()->GetPos();
    std::cout << "REAL POS: " << realposition << std::endl;
    std::cout << "SHOWN: " << dimensions[2] - slicePos << std::endl;
  }

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();  //update the render windows
}

void ArucoTestView::Start()
{
  //  m_VideoSource->SetVideoCameraInput(0);
  //  m_ArUcoTrackingDevice->setVideoSource(m_VideoSource);
  m_VideoSource->StartCapturing();

  int hertz = 1000;//m_Controls->UpdateRate->text().toInt();
  //  int updateTime = itk::Math::Round( static_cast<double>(1000.0/hertz) );
  int updateTime = static_cast<int>(static_cast<double>(1000.0/hertz));

  // resets the whole background
  m_VideoBackground->SetTimerDelay( updateTime );

  m_VideoBackground->AddRenderWindow( this->GetRenderWindowPart()->GetQmitkRenderWindow(QString("3d"))->GetVtkRenderWindow() );
  this->connect( m_VideoBackground, SIGNAL(NewFrameAvailable(mitk::VideoSource*)), this, SLOT(NewFrameAvailable(mitk::VideoSource*)));

  this->GetRenderWindowPart()->EnableDecorations(false, QStringList(mitk::IRenderWindowPart::DECORATION_BACKGROUND));
  m_VideoBackground->Enable();

  // Setup aruco
  //  TheIntrinsicFile="/home/riecker/Development/aruco/out_camera_data.xml";
  TheIntrinsicFile="/home/riecker/Development/src/Seminar/Plugins/org.mitk.gui.qt.aruco/out_camera_data.xml";
  if (TheIntrinsicFile!="") {
    TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
    //    TheCameraParameters.resize(m_VideoSource->GetImage().size());
    //    TheMarkerSize = 4;
  }
  //Configure other parameters
  if (ThePyrDownLevel>0)
    MDetector.pyrDown(ThePyrDownLevel);

  MDetector.detect(m_VideoSource->GetImage(), TheMarkers, TheCameraParameters, TheMarkerSize);
  TheInputImage = m_VideoSource->GetImage();
  for (unsigned int i=0;i<TheMarkers.size();i++) {
    TheMarkers[i].draw(TheInputImage,Scalar(0,0,255),1);
  }
}

#include <marker.h>

void ArucoTestView::NewFrameAvailable(mitk::VideoSource*)
{
  TheInputImage = m_VideoSource->GetImage(); /*
  //  cv::Mat* Image = new cv::Mat(m_VideoSource->GetCurrentFrame());

  //  IplImage* test;
  //  m_VideoSource->GetCurrentFrameAsOpenCVImage(test);
  //  cv::Mat im = test;
  //  cv::imshow("test",im);

  //  IplImage* x;
  //  x = const_cast<IplImage*>(m_VideoSource->GetCurrentFrame());
  //  cv::Mat im = x;
  //  cv::imshow("haha",im);

  //  cv::Point p;
  //  p.x = 0;
  //  p.y = 0;
  //  cv::circle(*Image,p,20,Scalar(255,0,0),3);
  //TheVideoCapturer.retrieve( TheInputImage);
  //copy image

  //  double tick = (double)getTickCount();//for checking the speed

  //Detection of markers in the image passed
  MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);

  for(int i=0; i<TheMarkers.size();i++){
    Marker marker = TheMarkers.at(i);
    if(marker.id == 900)
    {
      double pos[3];
      double orient[4];
      marker.OgreGetPoseParameters(pos,orient);

      focalPoint = pos;

      mitk::Point3D p;
      p[0]=marker.Tvec.at<double>(0,0);
      p[1]=marker.Tvec.at<double>(1,0);
      p[2]=marker.Tvec.at<double>(2,0);

      mitk::PointSet::Pointer ps = mitk::PointSet::New();
      ps->InsertPoint(0, p);

      //          mitk::DataNode::Pointer node = mitk::DataNode::New();
      //          node->SetName("TestCameraVector");
      //          node->SetData(ps);
      //          this->GetDataStorage()->Add(node);

      double position[3];
      double orientation[4];
      //hier kommen 3D koordinaten von der ogre methode ?! funktioniert nur für marker ?
      marker.OgreGetPoseParameters(position,orientation);

      markerPos = position;

      pTvec = marker.Tvec;
      pRvec = marker.Rvec;

      //          cv::Point3f tmp = TheCameraParameters.getCameraLocation(marker.Rvec, marker.Tvec);
      //          std::cout << "EYE : " << tmp.x << " " << tmp.y << " " << tmp.z << std::endl;

      //+ oder -
      camPos[0]=position[0]-marker.Tvec.at<double>(0,0);
      camPos[1]=position[1]-marker.Tvec.at<double>(1,0);
      camPos[2]=position[2]-marker.Tvec.at<double>(2,0);

      std::cout << "MarPos: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
      std::cout << "TvePos: " << marker.Tvec.at<double>(0,0) << " " << marker.Tvec.at<double>(0,1) << " " << marker.Tvec.at<double>(0,2) << std::endl;// Zugriff funktioniert so nicht!
      std::cout << "Control:" << marker.Tvec << std::endl;
      std::cout << "CamPos: " << camPos[0] << " " << camPos[1] << " " << camPos[2] << std::endl;

      cv::Rodrigues(marker.Rvec,ExtrinsicRotation);
      ExtrinsicTranslation = marker.Tvec;
      cv::Mat inv = ExtrinsicRotation.inv();

      cv::Mat focalvec = inv * ExtrinsicTranslation;

      focalPoint[0] = camPos[0] - focalvec.at<double>(0,0);
      focalPoint[1] = camPos[1] - focalvec.at<double>(1,0);
      focalPoint[2] = camPos[2] - focalvec.at<double>(2,0);

      //          std::cout << "MAT" << std::endl << ExtrinsicRotation << std::endl;
      //          std::cout << "INV" << std::endl << inv << std::endl;

      //          cv::Mat r = ExtrinsicRotation * ExtrinsicTranslation;

      //          camPos[0]=position[0]+r.at<double>(0,0);
      //          camPos[1]=position[1]+r.at<double>(1,0);
      //          camPos[2]=position[2]+r.at<double>(2,0);

      //          std::cout << "TRANS: " << ExtrinsicTranslation << std::endl;
      //          std::cout << "ROTAT: " << ExtrinsicRotation << std::endl;
      //          std::cout << "MAT: " << r << std::endl;

      ExtrinsicTransformation->SetElement(0,0,ExtrinsicRotation.at<double>(0,0));
      ExtrinsicTransformation->SetElement(1,0,ExtrinsicRotation.at<double>(1,0));
      ExtrinsicTransformation->SetElement(2,0,ExtrinsicRotation.at<double>(2,0));
      ExtrinsicTransformation->SetElement(0,1,ExtrinsicRotation.at<double>(0,1));
      ExtrinsicTransformation->SetElement(1,1,ExtrinsicRotation.at<double>(1,1));
      ExtrinsicTransformation->SetElement(2,1,ExtrinsicRotation.at<double>(2,1));
      ExtrinsicTransformation->SetElement(0,2,ExtrinsicRotation.at<double>(0,2));
      ExtrinsicTransformation->SetElement(1,2,ExtrinsicRotation.at<double>(1,2));
      ExtrinsicTransformation->SetElement(2,2,ExtrinsicRotation.at<double>(2,2));

      ExtrinsicTransformation->SetElement(0,3,ExtrinsicTranslation.at<double>(0,0)*(-1));
      ExtrinsicTransformation->SetElement(1,3,ExtrinsicTranslation.at<double>(1,0)*(-1));
      ExtrinsicTransformation->SetElement(2,3,ExtrinsicTranslation.at<double>(2,0)*(-1));

      ExtrinsicTransformation->SetElement(3,0,0);
      ExtrinsicTransformation->SetElement(3,1,0);
      ExtrinsicTransformation->SetElement(3,2,0);
      ExtrinsicTransformation->SetElement(3,3,1);

      //          ExtrinsicTransformation->Print(std::cout);

      //          std::cout << "ROTATION: " << ExtrinsicRotation << std::endl;
      //          std::cout << "TRANSLATION: " << ExtrinsicTranslation << std::endl;

      //          cv::Mat extrinsics;
      //          extrinsics.create(4,4,0.0);
      //          extrinsics
      //Kamera-Position ist MarkerPos + Translation * Rotation
    }
  }

  //check the speed by calculating the mean speed of all iterations
  //  AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
  //  AvrgTime.second++;
  //  cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

  TheInputImage.copyTo(TheInputImageCopy);
  for (unsigned int i=0;i<TheMarkers.size();i++) {
    cout<<"MARKER-NR:"<<TheMarkers[i]<<endl;
    //    TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
  }
  //print other rectangles that contains no valid markers
  /**     for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
  aruco::Marker m( MDetector.getCandidates()[i],999);
  m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
  }*/

//  if (  TheCameraParameters.isValid())
    //    for (unsigned int i=0;i<TheMarkers.size();i++) {
    //      CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
    //      CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
    //    }
//    cout<<endl<<endl;
  //    cv::imshow("in",TheInputImageCopy);
  //    cv::imshow("thres",MDetector.getThresholdedImage());
  /*
        mitk::NavigationData::Pointer navData = m_TrackingDeviceSource->GetOutput();
        mitk::Point3D pos = navData->GetPosition();
        mitk::BaseGeometry::Pointer geo = m_SelectedImageNode->GetData()->GetGeometry();
        mitk::Point3D indexPos;
        geo->WorldToIndex(pos,indexPos);
        cout << "POS: x:" << indexPos[0] << " y: " << indexPos[1] << " z: " << indexPos[2] << std::endl; */
}

void ArucoTestView::DoImageProcessing()
{
  TheVideoCapturer.open(-1);

  //check video is open
  if (!TheVideoCapturer.isOpened()) {
    cerr<<"Could not open video"<<endl;
    return;
  }

  //read first image to get the dimensions
  TheVideoCapturer>>TheInputImage;

  //read camera parameters if passed
  if (TheIntrinsicFile!="") {
    TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
    TheCameraParameters.resize(TheInputImage.size());
  }
  //Configure other parameters
  if (ThePyrDownLevel>0)
    MDetector.pyrDown(ThePyrDownLevel);

  //Create gui
  cv::namedWindow("thres",1);
  cv::namedWindow("in",1);
  MDetector.getThresholdParams( ThresParam1,ThresParam2);
  MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
  iThresParam1=ThresParam1;
  iThresParam2=ThresParam2;
  cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
  cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);

  char key=0;
  int index=0;
  //a single grab:
  TheVideoCapturer.grab();
  //capture until press ESC or until the end of the video
  //        while ( key!=27 && TheVideoCapturer.grab())
  {
    TheVideoCapturer.retrieve( TheInputImage);
    //copy image

    index++; //number of images captured
    double tick = (double)getTickCount();//for checking the speed
    //Detection of markers in the image passed
    MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);
    //chekc the speed by calculating the mean speed of all iterations
    AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
    AvrgTime.second++;
    cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

    //print marker info and draw the markers in image
    TheInputImage.copyTo(TheInputImageCopy);
    for (unsigned int i=0;i<TheMarkers.size();i++) {
      cout<<TheMarkers[i]<<endl;
      TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
    }
    //print other rectangles that contains no valid markers
    /**     for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
                aruco::Marker m( MDetector.getCandidates()[i],999);
                m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
            }*/

    //draw a 3d cube in each marker if there is 3d info
    if (  TheCameraParameters.isValid())
      for (unsigned int i=0;i<TheMarkers.size();i++) {
        CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
        CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
      }
    //DONE! Easy, right?
    cout<<endl<<endl<<endl;
    //show input with augmented information and  the thresholded image
    cv::imshow("in",TheInputImageCopy);
    cv::imshow("thres",MDetector.getThresholdedImage());
  }
}

void ArucoTestView::GetSliceFromMarkerPosition()
{
  // disables the 3D view for the ct image, because we need it for our
  // sliced plane which will only be visible in the 3D view?
  //  m_SelectedImageNode->SetVisibility(false, mitk::BaseRenderer::GetInstance
  //          ( mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget4")));
  m_SelectedImageNode->SetVisibility(false);

  mitk::Image::Pointer image = dynamic_cast<mitk::Image*>(m_SelectedImageNode->GetData());
  mitk::ExtractSliceFilter::Pointer sliceFilter = mitk::ExtractSliceFilter::New();

  //  mitk::PlaneGeometry::Pointer planeGeometry = mitk::PlaneGeometry::New();
  //  planeGeometry->SetImageGeometry(image);

  QmitkRenderWindow* renderWindow = this->GetRenderWindowPart()->GetQmitkRenderWindow("axial");
  //  unsigned int pos = renderWindow->GetSliceNavigationController()->GetSlice()->GetPos();
  mitk::PlaneGeometry::ConstPointer geometry = renderWindow->GetSliceNavigationController()->GetCurrentPlaneGeometry();

  //  mitk::SliceNavigationController::Pointer controller = mitk::SliceNavigationController::New();
  //  controller->SetInputWorldGeometry(image->GetGeometry());
  //  controller->SetViewDirection(mitk::SliceNavigationController::Axial);
  //  controller->Update();
  //  controller->GetSlice()->SetPos(pos);

  //  image->GetSlicedGeometry()->GetPlaneGeometry();

  //  mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget1");

  sliceFilter->SetInput(image);
  sliceFilter->SetWorldGeometry(geometry);
  sliceFilter->Update();

  mitk::Image::Pointer imageSlice = sliceFilter->GetOutput();

  m_SlicedImage->SetData(imageSlice);

  //  mitk::DataNode::Pointer slice = mitk::DataNode::New();
  //  slice->SetData(imageSlice);
  //  slice->SetName("Tracked Slice");
  //  slice->SetVisibility(true, mitk::BaseRenderer::GetInstance
  //          ( mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget1")));
  //  slice->SetVisibility(true, mitk::BaseRenderer::GetInstance
  //          ( mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget4")));
  //  this->GetDataStorage()->Add(slice);

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void cvTackBarEvents(int pos,void*)
{
  if (iThresParam1<3) iThresParam1=3;
  if (iThresParam1%2!=1) iThresParam1++;
  if (ThresParam2<1) ThresParam2=1;
  ThresParam1=iThresParam1;
  ThresParam2=iThresParam2;
  MDetector.setThresholdParams(ThresParam1,ThresParam2);
  //recompute
  MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
  TheInputImage.copyTo(TheInputImageCopy);
  for (unsigned int i=0;i<TheMarkers.size();i++)	TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
  //print other rectangles that contains no valid markers
  /*for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
        aruco::Marker m( MDetector.getCandidates()[i],999);
        m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
    }*/

  //draw a 3d cube in each marker if there is 3d info
  if (TheCameraParameters.isValid())
    for (unsigned int i=0;i<TheMarkers.size();i++)
      CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

  cv::imshow("in",TheInputImageCopy);
  cv::imshow("thres",MDetector.getThresholdedImage());
}

void ArucoTestView::CalibrateProbe()
{
  TheInputImage = m_VideoSource->GetImage();

  //! Board Detection from here
  BC.readFromFile("/home/riecker/Downloads/aruco_testproject/TestData/chessboardinfo_pix.yml");
  CP.readFromXMLFile("/home/riecker/Downloads/aruco_testproject/TestData/out_camera_data.xml");

  BDetector.setParams(BC,CP,100);
  BDetector.detect(TheInputImage);
  Board b = BDetector.getDetectedBoard();

  double boardPosTmp[3];
  double orientation[4];

  if(!b.empty())
  {
    b.OgreGetPoseParameters(boardPosTmp,orientation);
  }
  //! Board Detection until here

  mitk::NavigationData::Pointer navData = m_TrackingDeviceSource->GetOutput();
  mitk::Point3D probePos = navData->GetPosition();
  mitk::Quaternion orientation2 = navData->GetOrientation();

  //hier theoretisch noch die orientation holen und als rotation mitberechnen
  mitk::Point3D boardPos;
  boardPos[0]=boardPosTmp[0]; boardPos[1]=boardPosTmp[1]; boardPos[2]=boardPosTmp[2];

//  cout << "BOARD POSITION: " << boardPos[0] << " - " << boardPos[1] << " - " << boardPos[2] << endl;
//  cout << "MARKER POSITION: " << probePos[0] << " - " << probePos[1] << " - " << probePos[2] << endl;

  mitk::Vector3D offset = boardPos - probePos;

  mitk::Matrix3D rotaMat;
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
    {
      rotaMat[i][j] = orientation2.rotation_matrix_transpose()[i][j];
    }
  }

  mitk::Point3D tipPosition;
  for(int i=0;i<3;i++)
  {
      tipPosition[i] =  rotaMat[i][0] * offset[0] + rotaMat[i][1] * offset[1] + rotaMat[i][2] * offset[2];
  }

  m_ArUcoTrackingDevice->SetOffset(offset);
  m_ArUcoTrackingDevice->SetRotation(rotaMat);
  m_ArUcoTrackingDevice->SetTipMarkerProbePos(tipPosition);

  //Probe Pos isn correct here need update from navData but it works fine - placed an output into
  //the OnTimer function and its calibrated!
//  mitk::Point3D regisPos = navData->GetPosition();
  cout << "PROBE POSITION: " << probePos[0] << " - " << probePos[1] << " - " << probePos[2] << endl;
  cout << "BOARD POSITION: " << boardPos[0] << " - " << boardPos[1] << " - " << boardPos[2] << endl;

//  mitk::DataNode::Pointer testPositionNode = mitk::DataNode::New();
//  mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
//  pointSet->SetPoint(0,probePos);
//  testPositionNode->SetData(pointSet);
//  testPositionNode->SetName("Real_Position");
//  this->GetDataStorage()->Add(testPositionNode);

//  mitk::DataNode::Pointer testPositionNode2 = mitk::DataNode::New();
//  mitk::PointSet::Pointer pointSet2 = mitk::PointSet::New();
//  pointSet2->SetPoint(0,boardPos);
//  testPositionNode2->SetData(pointSet2);
//  testPositionNode2->SetName("Board_Position");
//  this->GetDataStorage()->Add(testPositionNode2);
}
