/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/

#include "mitkArUcoTrackingDevice.h"
#include <mitkIGTConfig.h>
//#include <mitkTimeStamp.h>
#include <itksys/SystemTools.hxx>
#include <iostream>
#include <itkMutexLockHolder.h>

typedef itk::MutexLockHolder<itk::FastMutexLock> MutexLockHolder;


mitk::ArUcoTrackingDevice::ArUcoTrackingDevice() :
  mitk::TrackingDevice(), m_GeoSet(false), m_TipPosSet(false), m_MarkerSize(-1), m_FirstGrabAfterOpening(true)
{
  // not added to TrackingDeviceList (which is static -> has to be refactored)
  TrackingDeviceData DeviceDataArUcoTracker = {VirtualTracker, "ArUco Tracker", "cube"};
  //set the type of this tracking device
  this->m_Data = DeviceDataArUcoTracker;

  m_TipMarkerProbePos[0]=0; m_TipMarkerProbePos[1]=0; m_TipMarkerProbePos[2]=0;

  m_Rotation.Fill(0);
  m_Rotation[0][0] = 1;
  m_Rotation[1][1] = 1;
  m_Rotation[2][2] = 1;

  m_VideoSource = mitk::OpenCVVideoSource::New();
  m_Offset.Fill(0.0);

  //only for intial test xxx
  this->InternalAddTool(mitk::ArUcoTool::New());

  this->m_MultiThreader = itk::MultiThreader::New();
  m_ThreadID = 0;
}


mitk::ArUcoTrackingDevice::~ArUcoTrackingDevice(){}


mitk::TrackingTool* mitk::ArUcoTrackingDevice::AddTool( const char* toolName, const char* fileName )
{
  // InternalTrackingTool: add using devices as friends - really necessary? -> probably refactor
  mitk::ArUcoTool::Pointer t = mitk::ArUcoTool::New();
  t->SetToolName("Aruco_Tool");
  return t.GetPointer();
}


bool mitk::ArUcoTrackingDevice::InternalAddTool(ArUcoTool::Pointer tool)
{
  m_AllTools.push_back(tool);
  return true;
}


bool mitk::ArUcoTrackingDevice::StartTracking()
{
  this->SetState(Tracking);            // go to mode Tracking
  this->m_StopTrackingMutex->Lock();  // update the local copy of m_StopTracking
  this->m_StopTracking = false;
  this->m_StopTrackingMutex->Unlock();

  m_TrackingFinishedMutex->Unlock(); // transfer the execution rights to tracking thread

  try {
    m_VideoSource->StartCapturing();
//    mitk::TimeStamp::GetInstance()->Start(this);
    m_ThreadID = m_MultiThreader->SpawnThread(this->ThreadStartTracking, this);    // start a new thread that executes the TrackTools() method

    return true;
  }
  catch(...)
  {
    m_ErrorMessage = "Error while trying to start the device!";
    return false;
  }
}


bool mitk::ArUcoTrackingDevice::StopTracking()
{
  Superclass::StopTracking();
  m_VideoSource->StopCapturing();
  return true;
}


unsigned int mitk::ArUcoTrackingDevice::GetToolCount() const
{
  return (unsigned int)this->m_AllTools.size();
}


mitk::TrackingTool* mitk::ArUcoTrackingDevice::GetTool(unsigned int toolNumber) const
{
  if ( toolNumber >= this->GetToolCount())
    return NULL;
  else
    return this->m_AllTools[toolNumber];
}

//brauche hier einen setter für m_VideoSource und setze es dann auf den m_VideoSource aus
//der ArucoTestView somit benutzen alle den selben m_VideoSource

//void mitk::ArUcoTrackingDevice::setVideoSource(mitk::OpenCVVideoSource::Pointer source)
//{
//    this->m_VideoSource = source;
//}

bool mitk::ArUcoTrackingDevice::OpenConnection()
{
  try {
    m_FirstGrabAfterOpening = true;
//    m_VideoSource->SetVideoCameraInput(0); //vor Methodenaufruf muss unbedingt die VideoSource gesetzt werden!
    m_CameraParameters.readFromXMLFile("/home/riecker/Development/src/Seminar/Plugins/org.mitk.gui.qt.aruco/out_camera_data.xml");
    m_MarkerSize = 4;
    this->SetState(Ready);
    return true;
  }
  catch(...)
  {
    // m_VideoSource->Reset(); is private - why? Change?
    this->SetState(Setup);
    m_ErrorMessage = "Error while trying to open connection to the camera for ArUco tracking.";
    return false;
   }
}


bool mitk::ArUcoTrackingDevice::CloseConnection()
{
  bool returnValue = true;
  if (this->GetState() == Setup)
    return true;

  try {
    m_VideoSource->StopCapturing();
  }
  catch (...) {
    returnValue = false;
  }
  this->SetState(Setup);
  return returnValue;
}

std::vector<mitk::ArUcoTool::Pointer> mitk::ArUcoTrackingDevice::GetAllTools()
{
  return this->m_AllTools;
}

void mitk::ArUcoTrackingDevice::SetTipMarkerProbePos(mitk::Point3D pos)
{
    m_TipMarkerProbePos = pos;
    m_TipPosSet = true;
}

cv::Mat grabbedImageCopy;

void mitk::ArUcoTrackingDevice::TrackTools()
{
  try
  {
    /* lock the TrackingFinishedMutex to signal that the execution rights are now transfered to the tracking thread */
    MutexLockHolder trackingFinishedLockHolder(*m_TrackingFinishedMutex); // keep lock until end of scope

    bool localStopTracking;       // Because m_StopTracking is used by two threads, access has to be guarded by a mutex. To minimize thread locking, a local copy is used here
    this->m_StopTrackingMutex->Lock();  // update the local copy of m_StopTracking
    localStopTracking = this->m_StopTracking;
    this->m_StopTrackingMutex->Unlock();

    while ((this->GetState() == Tracking) && (localStopTracking == false))
    {
      cv::Mat grabbedImage;
      m_VideoSource->FetchFrame();
      if(m_VideoSource->GetCurrentFrame() != NULL)
      {
        grabbedImage = m_VideoSource->GetImage();

        if ((m_FirstGrabAfterOpening) && m_CameraParameters.isValid())
        {
          m_CameraParameters.resize(grabbedImage.size());
          m_FirstGrabAfterOpening = false;
        }

        m_MarkerSize = 200;

        vector<aruco::Marker> markers;
        m_MarkerDetector.detect(grabbedImage,markers,m_CameraParameters,m_MarkerSize);
//        std::cout << markers.size() << std::endl;
        if(markers.size()>0)
        {
          for(int i=0; i< markers.size(); ++i)
          {
              aruco::Marker marker = markers.at(i);
//              cout << "Marker id: " << marker.id << endl;
              //! Als Tracking Marker wird nur noch der Marker mit der ID 900 genommen,
              //! damit es nicht zum Chaos kommt mit den Markern vom Board beim
              //! kalibrieren der Sonde
              if(marker.id == 900)
              {
                double position[3];
                double orientation[4];
                //hier kommen 3D koordinaten von der ogre methode ?! funktioniert nur für marker ?
                marker.OgreGetPoseParameters(position,orientation);

//       !         mitk::Point3D mitkpoint;
//       !         mitk::FillVector3D(mitkpoint, position[0], position[1], position[2]);

                // TODO evaluieren ob das so funktioniert
//                mitkpoint[0]/=5; mitkpoint[1]/=5; mitkpoint[2]/=5;

//                position[0]+=m_Offset[0]; position[1]+=m_Offset[1]; position[2]+=m_Offset[2];

//                mitk::Point3D final;
//                for(int i=0;i<3;i++)
//                {
//                    final[i] =  m_Rotation[i][0] * position[0] + m_Rotation[i][1] * position[1] + m_Rotation[i][2] * position[2];
//                }

//       !         mitk::Vector3D offsetPosition;// = tmp + m_Offset;
//       !         mitk::FillVector3D(offsetPosition,mitkpoint[0],mitkpoint[1],mitkpoint[2]);

                //[0 2 3 1] oder [1 2 3 0] oder [3 0 1 2]
                mitk::Quaternion mitkorientation(orientation[1], orientation[2], orientation[3], orientation[0]);

                if(m_TipPosSet)
                {
                    mitk::Matrix3D rotaMat;
                    for(int i=0; i<3; i++)
                    {
                      for(int j=0; j<3; j++)
                      {
                        rotaMat[i][j] = mitkorientation.rotation_matrix_transpose().transpose()[i][j];
                      }
                    }

                    mitk::Point3D tipPosition;
                    for(int i=0;i<3;i++)
                    {
                        tipPosition[i] =  rotaMat[i][0] * m_TipMarkerProbePos[0] + rotaMat[i][1] * m_TipMarkerProbePos[1] + rotaMat[i][2] * m_TipMarkerProbePos[2];
                    }

                    tipPosition[0]+=position[0]; tipPosition[1]+=position[1]; tipPosition[2]+=position[2];
//                    this->m_AllTools[0]->SetPosition(position);
                    this->m_AllTools[0]->SetToolTip(tipPosition);
                    std::cout << "TIP-POSITION: X:" << tipPosition[0] << " Y: " << tipPosition[1] << " Z: " << tipPosition[2] << std::endl;
                }
                else
                {
                    this->m_AllTools[0]->SetPosition(position);
                    std::cout << "TOOL-POSITION: X:" << position[0] << " Y: " << position[1] << " Z: " << position[2] << std::endl;
                }

                this->m_AllTools[0]->SetOrientation(mitkorientation);
                this->m_AllTools[0]->SetDataValid(true);
              }
          }

        }
        //print marker info and draw the markers in image
//        grabbedImage.copyTo(grabbedImageCopy);
//        for (unsigned int i=0;i<markers.size();i++) {
//          cout<<markers[i]<<endl;
//          markers[i].draw(grabbedImageCopy,cv::Scalar(0,0,255),1);
//        }

        //std::vector<mitk::ArUcoTool::Pointer> detectedTools = this->DetectTools();
        //std::vector<mitk::ArUcoTool::Pointer> allTools = this->GetAllTools();
        //std::vector<mitk::ArUcoTool::Pointer>::iterator itAllTools;
        //for(itAllTools = allTools.begin(); itAllTools != allTools.end(); itAllTools++)
        //{
        //  mitk::ArUcoTool::Pointer currentTool = *itAllTools;
        //  //test if current tool was detected
        //  std::vector<mitk::ArUcoTool::Pointer>::iterator itDetectedTools;
        //  bool foundTool = false;
        //  for(itDetectedTools = detectedTools.begin(); itDetectedTools != detectedTools.end(); itDetectedTools++)
        //  {
        //    mitk::ArUcoTool::Pointer aktuDet = *itDetectedTools;
        //    std::string tempString(currentTool->GetCalibrationName());
        //    if (tempString.compare(aktuDet->GetCalibrationName())==0)
        //    {
        //      currentTool->SetToolHandle(aktuDet->GetToolHandle());
        //      foundTool = true;
        //    }
        //  }
        //  if (!foundTool)
        //  {
        //    currentTool->SetToolHandle(0);
        //  }

        //  if (currentTool->GetToolHandle() != 0)
        //  {
        //    currentTool->SetDataValid(true);
        //    //get tip position of tool:
        //    std::vector<double> pos_vector = this->GetDevice()->GetTipPosition(currentTool->GetToolHandle());
        //    //write tip position into tool:
        //    mitk::Point3D pos;
        //    pos[0] = pos_vector[0];
        //    pos[1] = pos_vector[1];
        //    pos[2] = pos_vector[2];
        //    currentTool->SetPosition(pos);
        //    //get tip quaternion of tool
        //    std::vector<double> quat = this->GetDevice()->GetTipQuaternions(currentTool->GetToolHandle());
        //    //write tip quaternion into tool
        //    mitk::Quaternion orientation(quat[1], quat[2], quat[3], quat[0]);
        //    currentTool->SetOrientation(orientation);
        //  }
        //  else
        //  {
        //    mitk::Point3D origin;
        //    origin.Fill(0);
        //    currentTool->SetPosition(origin);
        //    currentTool->SetOrientation(mitk::Quaternion(0,0,0,0));
        //    currentTool->SetDataValid(false);
        //  }
        //}
        /* Update the local copy of m_StopTracking */
        this->m_StopTrackingMutex->Lock();
        localStopTracking = m_StopTracking;
        this->m_StopTrackingMutex->Unlock();
      }
    }
  }
  catch(...)
  {
    this->StopTracking();
//    this->SetErrorMessage("Error while trying to track tools. Thread stopped.");
  }
}


ITK_THREAD_RETURN_TYPE mitk::ArUcoTrackingDevice::ThreadStartTracking(void* pInfoStruct)
{
  /* extract this pointer from Thread Info structure */
  struct itk::MultiThreader::ThreadInfoStruct * pInfo = (struct itk::MultiThreader::ThreadInfoStruct*)pInfoStruct;
  if (pInfo == NULL)
  {
    return ITK_THREAD_RETURN_VALUE;
  }
  if (pInfo->UserData == NULL)
  {
    return ITK_THREAD_RETURN_VALUE;
  }
  ArUcoTrackingDevice *trackingDevice = (ArUcoTrackingDevice*)pInfo->UserData;

  if (trackingDevice != NULL)
    trackingDevice->TrackTools();

  return ITK_THREAD_RETURN_VALUE;
}

void mitk::ArUcoTrackingDevice::SetCameraParameters(const aruco::CameraParameters & cameraParameters)
{
  m_CameraParameters = cameraParameters;
}

const aruco::CameraParameters & mitk::ArUcoTrackingDevice::GetCameraParameters() const
{
  return m_CameraParameters;
}

aruco::MarkerDetector& mitk::ArUcoTrackingDevice::GetMarkerDetector()
{
  return m_MarkerDetector;
}

const aruco::MarkerDetector& mitk::ArUcoTrackingDevice::GetMarkerDetector() const
{
  return m_MarkerDetector;
}

void mitk::ArUcoTrackingDevice::SetPointGeometry(mitk::BaseGeometry *geo)
{
    this->m_Geometry = geo;
    this->m_GeoSet = true;
}

