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


#ifndef ArucoTestView_h
#define ArucoTestView_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_ArucoTestViewControls.h"

#include <mitkOpenCVVideoSource.h>
#include <mitkTrackingDeviceSource.h>
#include <mitkArUcoTrackingDevice.h>
#include <mitkNavigationToolStorage.h>
#include <mitkNavigationDataObjectVisualizationFilter.h>

#include <QmitkVideoBackground.h>
#include <QmitkRenderWindow.h>

#include <mitkImage.h>


class ArucoTestView : public QmitkAbstractView
{
  Q_OBJECT

  public:

    ArucoTestView();
    virtual ~ArucoTestView();

    static const std::string VIEW_ID;

    virtual void CreateQtPartControl(QWidget *parent);

  protected slots:

    void SetupArUcoTracker();

    void OnTimer();

    void DoImageProcessing();

    void Start();

    void NewFrameAvailable(mitk::VideoSource*);

    void GetSliceFromMarkerPosition();

    void SetPermanentSlicing(bool);

    void CalibrateProbe();

    void SetImageGeo();

    void SetRefImage();

    void CameraTest();

    void CamParamsTest();

    void SetTransformation();

    void TestSliceSelector();

    void GeoBugTest();

  protected:

    virtual void SetFocus();

    /**
     * @brief called by QmitkFunctionality when DataManager's selection has changed
     */
    virtual void OnSelectionChanged( berry::IWorkbenchPart::Pointer source, const QList<mitk::DataNode::Pointer>& nodes );

    Ui::ArucoTestViewControls m_Controls;

    /**
     * @brief grabbing a webcam stream via OpenCV
     */
    mitk::OpenCVVideoSource::Pointer m_VideoSource;

    /**
     * @brief placing the video stream in the background of a renderwindow
     */
    QmitkVideoBackground* m_VideoBackground;

    /**
     * @brief represents a monoscopic tracking device based on the ArUco maker detector
     */
    mitk::ArUcoTrackingDevice::Pointer m_ArUcoTrackingDevice;

    /**
     * @brief Connects a mitk::TrackingDevice to a MITK-IGT NavigationData-Filterpipeline
     */
    mitk::TrackingDeviceSource::Pointer m_TrackingDeviceSource;

    /**
     * @brief represents a collection of navigation tools
     */
    mitk::NavigationToolStorage::Pointer m_ToolStorage;

    /**
     * @brief visualization filter uses output from m_Source
     */
    mitk::NavigationDataObjectVisualizationFilter::Pointer m_Visualizer;

    QTimer* m_Timer;

  private:

    mitk::DataNode::Pointer m_SelectedImageNode;
    mitk::DataNode::Pointer m_SlicedImage;
    mitk::Image::Pointer m_RefImage;
    mitk::AffineTransform3D::Pointer m_Transformation;

    bool m_Slicing = false;

};

#endif // ArucoTestView_h

