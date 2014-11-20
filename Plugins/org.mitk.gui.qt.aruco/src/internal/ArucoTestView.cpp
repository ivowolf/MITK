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

#include "cvdrawingutils.h"

using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
cv::Mat TheInputImageCopy;
aruco::MarkerDetector MDetector;

aruco::BoardDetector BDetector;

cv::VideoCapture TheVideoCapturer;
vector<aruco::Marker> TheMarkers;
aruco::CameraParameters TheCameraParameters;

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;

cv::Mat TheInputImage;
void cvTackBarEvents(int pos,void*);

const std::string ArucoTestView::VIEW_ID = "org.mitk.views.arucotestview";

ArucoTestView::ArucoTestView()
: m_VideoSource(0), m_Timer(NULL)
{
  m_TrackingDeviceSource = mitk::TrackingDeviceSource::New();
  m_ArUcoTrackingDevice = mitk::ArUcoTrackingDevice::New();
  m_ToolStorage = mitk::NavigationToolStorage::New();
  m_SelectedImageNode = mitk::DataNode::New();
  m_SlicedImage = mitk::DataNode::New();
  m_SlicedImage->SetName("SlicedImage");
  m_SlicedImage->SetVisibility(true, mitk::BaseRenderer::GetInstance
          ( mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget1")));
  m_SlicedImage->SetVisibility(true, mitk::BaseRenderer::GetInstance
          ( mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget4")));
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

  // retrieve old preferences
  m_VideoSource = mitk::OpenCVVideoSource::New();
  m_VideoBackground = new QmitkVideoBackground(m_VideoSource);
  m_VideoBackground->setParent(parent);

  m_VideoSource->SetVideoCameraInput(0);
  m_ArUcoTrackingDevice->SetVideoSource(m_VideoSource);
}

void ArucoTestView::SetPermanentSlicing(bool slicing)
{
  m_Slicing = slicing;
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
  m_TrackingDeviceSource->SetTrackingDevice(m_ArUcoTrackingDevice);
  m_TrackingDeviceSource->RegisterAsMicroservice();
  m_ToolStorage->RegisterAsMicroservice(m_TrackingDeviceSource->GetMicroserviceID());

  this->GetDataStorage()->Add(m_SlicedImage);

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
  scale.Fill(2);
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
  mitk::RenderingManager::GetInstance()->InitializeViewsByBoundingObjects(this->GetDataStorageReference()->GetDataStorage());
}

void ArucoTestView::OnTimer()
{
  //Here we call the Update() method from the Visualization Filter. Internally the filter checks if
  //new NavigationData is available. If we have a new NavigationData the cone position and orientation
  //will be adapted.
  m_Visualizer->Update();

  if(m_Slicing)
  {
    this->GetSliceFromMarkerPosition();
  }

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();  //update the render windows
}

void ArucoTestView::Start()
{
//  m_VideoSource->SetVideoCameraInput(0);
//  m_ArUcoTrackingDevice->setVideoSource(m_VideoSource);
  m_VideoSource->StartCapturing();

  int hertz = 200;//m_Controls->UpdateRate->text().toInt();
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
    TheCameraParameters.resize(m_VideoSource->GetImage().size());
    TheMarkerSize = 4;
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

BoardConfiguration BC;
CameraParameters CP;

void ArucoTestView::NewFrameAvailable(mitk::VideoSource*)
{
  TheInputImage = m_VideoSource->GetImage();
  //TheVideoCapturer.retrieve( TheInputImage);
  //copy image

//  double tick = (double)getTickCount();//for checking the speed

  //Detection of markers in the image passed
  MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);

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

  if (  TheCameraParameters.isValid())
//    for (unsigned int i=0;i<TheMarkers.size();i++) {
//      CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
//      CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
//    }
    cout<<endl<<endl;
    cv::imshow("in",TheInputImageCopy);
//    cv::imshow("thres",MDetector.getThresholdedImage());
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

    if(!b.empty())
    {
        double orientation[4];
        b.OgreGetPoseParameters(boardPosTmp,orientation);
        cout << "Position: " << boardPosTmp[0] << " - " << boardPosTmp[1] << " - " << boardPosTmp[2] << endl;
    }
    //! Board Detection until here

    mitk::NavigationData::Pointer navData = m_TrackingDeviceSource->GetOutput();
    mitk::Point3D probePos = navData->GetPosition();
    mitk::Point3D boardPos;
    boardPos[0]=boardPosTmp[0]; boardPos[1]=boardPosTmp[1]; boardPos[2]=boardPosTmp[2];

    cout << "BOARD POSITION: " << boardPos[0] << " - " << boardPos[1] << " - " << boardPos[2] << endl;
    cout << "MARKER POSITION: " << probePos[0] << " - " << probePos[1] << " - " << probePos[2] << endl;

    // TODO Evaluieren ob das Ergebnis so stimmt
    mitk::Vector3D offset = boardPos - probePos;

    m_ArUcoTrackingDevice->SetOffset(offset);
}
