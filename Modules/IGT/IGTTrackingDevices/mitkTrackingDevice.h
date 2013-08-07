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


#ifndef MITKTRACKINGDEVICE_H_HEADER_INCLUDED_C1C2FCD2
#define MITKTRACKINGDEVICE_H_HEADER_INCLUDED_C1C2FCD2

#include <MitkIGTExports.h>
#include "itkObject.h"
#include "mitkCommon.h"
#include "mitkTrackingTypes.h"
#include "itkFastMutexLock.h"


namespace mitk {
    class TrackingTool; // interface for a tool that can be tracked by the TrackingDevice

    /**Documentation
    * \brief Interface for all Tracking Devices
    *
    * Defines the methods that are common for all tracking devices.
    *
    *  \ingroup IGT
    */
    class MitkIGT_EXPORT TrackingDevice : public itk::Object
    {
    public:
      mitkClassMacro(TrackingDevice, itk::Object);

      /** Defines the rotation modes of this tracking device which results in different representations
       *  of quaternions.
       *  - Standard: normal representation, rawdata from the device is not changed
       *  - Transposed: the rotation is stored transposed, which is standard in VNL
       *  ndiStandard is default
       *  use SetRotationMode to change the mode.
       */
      enum RotationMode {RotationStandard, RotationTransposed};

      enum TrackingDeviceState {Setup, Ready, Tracking};   ///< Type for state variable. The trackingdevice is always in one of these states
      /**
       * \brief Opens a connection to the device
       *
       * This may only be called if there is currently no connection to the device.
       * If OpenConnection() is successful, the object will change from Setup state to Ready state
       */
      virtual bool OpenConnection() = 0;

      /**
       * \brief Closes the connection to the device
       *
       * This may only be called if there is currently a connection to the device, but tracking is
       * not running (e.g. object is in Ready state)
       */
      virtual bool CloseConnection() = 0; ///< Closes the connection with the device

      /**
       * \brief start retrieving tracking data from the device.
       *
       * This may only be called after the connection to the device has been established
       * with a call to OpenConnection() (E.g. object is in Ready mode). This will change the
       * object state from Ready to Tracking
       */
      virtual bool StartTracking() = 0;

      /**
       * \brief stop retrieving tracking data from the device.
       * stop retrieving tracking data from the device.
       * This may only be called after StartTracking was called
       * (e.g. the object is in Tracking mode).
       * This will change the object state from Tracking to Ready.
       */
      virtual bool StopTracking();

      /**
       * \brief Return tool with index toolNumber
       *
       * tools are numbered from 0 to GetToolCount() - 1.
       */
      virtual TrackingTool* GetTool(unsigned int toolNumber) const = 0;

      /**
      * \brief Returns the tool with the given tool name
      *
      * Note: subclasses can and should implement optimized versions of this method
      * \return the given tool or NULL if no tool with that name exists
      */
      virtual mitk::TrackingTool* GetToolByName(std::string name) const;

      /**
       * \brief Returns number of tracking tools
       */
      virtual unsigned int GetToolCount() const = 0;

      /** Sets the rotation mode of this class. See documentation of enum RotationMode for details
       *  on the different modes. This method has to be implemented in a deriving class to become
       *  functional / if different rotation modes should be supported.
       */
      virtual void SetRotationMode(RotationMode r);

     /** @return Returns the rotation mode of this class. See documentation of enum
       *         RotationMode for details on the different modes.
       */
      itkGetConstMacro(RotationMode,RotationMode);

      /**
       * \brief return current error message
       */
      itkGetStringMacro(ErrorMessage);
      /**
       * \brief return current object state (Setup, Ready or Tracking)
       */
      TrackingDeviceState GetState() const;

      /**
       * \brief Deprecated! Use the more specific getDeviceData instead. return device type identifier
       */
    TrackingDeviceType GetType() const;
       /**
       * \brief Deprecated! Use the more specific setDeviceData instead. set device type
       */
    void SetType(TrackingDeviceType type);

     /**
       * \brief return device data
     */
    TrackingDeviceData GetData() const;
       /**
       * \brief set device type
       */
    void SetData(TrackingDeviceData data);

    protected:
      /**
      * \brief  set error message
      */
      itkSetStringMacro(ErrorMessage);
      /**
      * \brief  change object state
      */
      void SetState(TrackingDeviceState state);


      TrackingDevice();
      virtual ~TrackingDevice();

    TrackingDeviceData m_Data; ///< current device Data
      TrackingDeviceState m_State; ///< current object state (Setup, Ready or Tracking)
      bool m_StopTracking;       ///< signal stop to tracking thread
      itk::FastMutexLock::Pointer m_StopTrackingMutex; ///< mutex to control access to m_StopTracking
      itk::FastMutexLock::Pointer m_TrackingFinishedMutex; ///< mutex to manage control flow of StopTracking()
      itk::FastMutexLock::Pointer m_StateMutex; ///< mutex to control access to m_State
      std::string m_ErrorMessage; ///< current error message
      RotationMode m_RotationMode; ///< defines the rotation mode Standard or Transposed, Standard is default
    };
} // namespace mitk

#endif /* MITKTRACKINGDEVICE_H_HEADER_INCLUDED_C1C2FCD2 */
