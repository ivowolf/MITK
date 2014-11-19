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

#ifndef MITK_MeasurementsPoints_H
#define MITK_MeasurementsPoints_H

#include "mitkMeasurementsPoint.h"

#include <mitkBaseData.h>
#include <mitkCommon.h>
#include <MitkEndoscopyCalibrationExports.h>
#include <vtkSmartPointer.h>


#include <mitkPointSet.h>
#include <mitkDataNode.h>
#include <mitkSurface.h>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkStringArray.h>
#include <vtkPolyData.h>
#include <utility>

namespace mitk
{
  /*!
  \brief mitk::MeasurementsPoints - This class is the base class for the individual measurement procedures

  \details This class class is the base class for mitkPreoperativeAnnulusPoints, mitkPreoperativePoints,
  mitkPreoperativeSegmentAnalysis. It stores important member variables that are used in the child classes.
  Furthermore, it provides a routine to save and load files.


  */
  class MITK_ENDOSCOPYCALIBRATION_EXPORT MeasurementsPoints : public BaseData
  {
  public:
    mitkClassMacro(MeasurementsPoints, BaseData);
    itkFactorylessNewMacro(Self)
      itkCloneMacro(Self)

      ///@{
      /**
      *   \brief inherited from base class
      */
      virtual void UpdateOutputInformation();
    virtual void SetRequestedRegionToLargestPossibleRegion();
    virtual bool RequestedRegionIsOutsideOfTheBufferedRegion();
    virtual bool VerifyRequestedRegion();
    virtual void SetRequestedRegion(const itk::DataObject *data );

    itkGetMacro(NumberOfPoints, int );
    itkGetMacro(NameOfSchemeFile, std::string );
    itkGetMacro(PointSet, mitk::PointSet::Pointer);
    itkGetMacro(PointSetNode, mitk::DataNode::Pointer);
    /**
    *   \brief Returns the measurement point at position i ( index i starts at 0 )
    */
    mitk::MeasurementsPoint* GetMPointAtPosition(int i);
    /**
    *   \brief Synchronizes the mitk::MeasurementsPoint* vector with the mitk::Pointset and the vtk::Points.
    */
    void UpdatePointSet();

    /**
    *   \brief Synchronizes 2D and 3D selection depending on the selection status of mitk::MeasurementsPoint*.
    *   The status of the point in the pointset is set to "selected" and highlighted in the render window.
    */
    void SetActivePointSelected();
    /**
    *   \brief Fills the vtkStringArray with the names of the points
    */
    void FillPointNames();


  protected:
    MeasurementsPoints();
    ~MeasurementsPoints();


    /**
    *   \brief Defines the MeasurementsPoint* instances that should be measured. Reimplemented in subclasses.
    */
    virtual void defineAllMeasurementPoints(){};
    // vector that holds all measurement point objects that belong to the scheme
    std::vector<mitk::MeasurementsPoint*> m_MeasurementsPoints;

    // number of measurement points belonging to the scheme
    int m_NumberOfPoints;

    // connection between points
    //vtkSmartPointer<vtkCellArray> m_CellArray;

    // points and connections have to be saved in VTK's *.vtp format (polydata)
    // to store the additional names for each point
    vtkSmartPointer<vtkPoints> m_Points;
    // saves a name for each point and assigns it to a vtk polydata
    vtkSmartPointer<vtkStringArray> m_PointNames;
    // mitk point set and its node for displaying and saving the measurement points
    mitk::PointSet::Pointer m_PointSet;
    mitk::DataNode::Pointer m_PointSetNode;

    // filename of measurement scheme
    std::string m_NameOfSchemeFile;
  };
}

# endif //!MITK_MeasurementsPoints_H