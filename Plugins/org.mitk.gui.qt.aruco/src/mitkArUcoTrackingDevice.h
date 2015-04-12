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

#ifndef MITKARUCOTRACKINGDEVICE_H_HEADER_INCLUDED_
#define MITKARUCOTRACKINGDEVICE_H_HEADER_INCLUDED_


#include <vector>
#include <mitkIGTConfig.h>
#include <mitkTrackingDevice.h>
#include "mitkArUcoTool.h"
#include <mitkInternalTrackingTool.h>
#include <mitkTrackingTool.h>
#include <mitkBaseGeometry.h>

#include <itkMultiThreader.h>

#include <mitkOpenCVVideoSource.h>
#include <aruco.h>

#include <org_mitk_gui_qt_aruco_Export.h>

namespace mitk
{
  /** Documentation:
  *   \brief An object of this class represents a monoscopic tracking device based on the ArUco maker detector.
  *
  *          You can add tools to this device, then open the connection and start tracking.
  *          The tracking device will then continuously update the tool coordinates.
  *   \ingroup IGT
  */
  class ARUCO_EXPORT ArUcoTrackingDevice : public TrackingDevice
  {
  public:

    mitkClassMacro(ArUcoTrackingDevice, TrackingDevice)
    itkNewMacro(Self)

//    void setVideoSource(mitk::OpenCVVideoSource::Pointer source);

    itkSetMacro(VideoSource, mitk::OpenCVVideoSource::Pointer)
    itkGetMacro(VideoSource, mitk::OpenCVVideoSource::Pointer)

    itkSetMacro(Offset,mitk::Vector3D)
    itkGetMacro(Offset,mitk::Vector3D)

    itkGetMacro(Rotation, mitk::Matrix3D)
    itkSetMacro(Rotation, mitk::Matrix3D)

    void SetPointGeometry(mitk::BaseGeometry* geo);

    void SetTipMarkerProbePos(mitk::Point3D pos);

    /**
    * \brief Starts the tracking.
    * \return Returns true if the tracking is started. Returns false if there was an error.
    */
    virtual bool StartTracking();

    /**
    * \brief Stops the tracking.
    * \return Returns true if the tracking is stopped. Returns false if there was an error.
    */
    virtual bool StopTracking();

    /**
    * \brief Opens the connection to the device. This have to be done before the tracking is startet.
    */
    virtual bool OpenConnection();

    /**
    * \brief Closes the connection and clears all resources.
    */
    virtual bool CloseConnection();

    /**
    * \return Returns the number of tools which have been added to the device.
    */
    virtual unsigned int GetToolCount() const;

    /**
    * \param toolNumber The number of the tool which should be given back.
    * \return Returns the tool which the number "toolNumber". Returns NULL, if there is
    * no tool with this number.
    */
    TrackingTool* GetTool(unsigned int toolNumber)  const;


    /**
    * \brief Create a new ArUco tool with toolName and fileName and add it to the list of tools
    *
    * This method will create a new ArUcoTool object, load the tool definition file fileName,
    * set the tool name toolName and then add it to the list of tools.
    * It returns a pointer of type mitk::TrackingTool to the tool
    * that can be used to read tracking data from it.
    * This is the only way to add tools to ArUcoTrackingDevice.
    *
    * \warning adding tools is not possible in tracking mode, only in setup and ready.
    */
    mitk::TrackingTool* AddTool(const char* toolName, const char* fileName);


    /** @brief Sets the camera parameters. */
    void SetCameraParameters(const aruco::CameraParameters & cameraParameters);

    /** @brief Gets the camera parameters. */
    const aruco::CameraParameters & GetCameraParameters() const;

    /** @brief Gets the ArUco markerdetector (with write access to allow configuration)*/
    aruco::MarkerDetector& GetMarkerDetector();

    /** @brief Gets the ArUco markerdetector */
    const aruco::MarkerDetector& GetMarkerDetector() const;

  protected:
    ArUcoTrackingDevice();
    ~ArUcoTrackingDevice();

    /**
    * \brief Adds a tool to the tracking device.
    *
    * \param tool  The tool which will be added.
    * \return Returns true if the tool has been added, false otherwise.
    */
    bool InternalAddTool(ArUcoTool::Pointer tool);

    /**
    * \brief This method tracks tools as long as the variable m_Mode is set to "Tracking".
    * Tracking tools means grabbing frames from the camera an updating the tools.
    */
    void TrackTools();

    /**
    * \return Returns all tools of the tracking device.
    */
    std::vector<ArUcoTool::Pointer> GetAllTools();

    static ITK_THREAD_RETURN_TYPE ThreadStartTracking(void* data);

    std::vector<ArUcoTool::Pointer> m_AllTools; ///< vector holding all tools
    itk::MultiThreader::Pointer m_MultiThreader;
    int m_ThreadID;

    /** \brief The directory where the tool calibration files can be found */
    std::string m_ToolfilesDir;

    // ArUco specific
    mitk::OpenCVVideoSource::Pointer m_VideoSource;
    aruco::CameraParameters m_CameraParameters;
    aruco::MarkerDetector m_MarkerDetector;

    mitk::BaseGeometry* m_Geometry;
    bool m_GeoSet = false;

    mitk::Point3D m_TipMarkerProbePos;
    bool m_TipPosSet = false;

    float m_MarkerSize;
    string m_ErrorMessage;
    mitk::Vector3D m_Offset;

    mitk::Matrix3D m_Rotation;

    bool m_FirstGrabAfterOpening;
  };
}//mitk
#endif /* MITKARUCOTRACKINGDEVICE_H_HEADER_INCLUDED_ */
