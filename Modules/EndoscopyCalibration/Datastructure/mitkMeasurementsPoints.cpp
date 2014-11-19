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

#include "mitkMeasurementsPoints.h"
#include <mitkPointSetVtkMapper2D.h>
#include <mitkPointSetVtkMapper3D.h>
#include <mitkSurfaceVtkMapper3D.h>
#include <mitkSurfaceGLMapper2D.h>
#include <mitkStandardFileLocations.h>
#include <mitkPointSetWriter.h>

// Qt
#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>

// Vtk
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSplineFilter.h>
#include <vtkKochanekSpline.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkPointData.h>
#include <vtkTubeFilter.h>

namespace mitk
{
  MeasurementsPoints::MeasurementsPoints():
m_NumberOfPoints(-1),
m_NameOfSchemeFile("")
{
  m_PointSet = mitk::PointSet::New();
  m_MeasurementsPoints.clear();

  m_PointSetNode = mitk::DataNode::New();
  m_PointSetNode->SetData(m_PointSet);
  m_PointSetNode->SetIntProperty("point 2D size", 1);
  m_PointSetNode->SetFloatProperty("pointsize", 1.0f);
  m_PointSetNode->SetMapper(1, mitk::PointSetVtkMapper2D::New());
  m_PointSetNode->SetMapper(2, mitk::PointSetVtkMapper3D::New());
  m_PointSetNode->SetBoolProperty("show label", true);
  m_PointSetNode->SetStringProperty("label","");
  // vtk objects
  // points for polydata
  m_Points = vtkSmartPointer<vtkPoints>::New();
  m_Points->SetDataTypeToFloat();
  m_Points->Reset();

  m_PointNames = vtkSmartPointer<vtkStringArray>::New();
  m_PointNames->SetName("PointNames");
  m_PointNames->SetNumberOfComponents(1);

}

MeasurementsPoints::~MeasurementsPoints()
{
}

void MeasurementsPoints::SetActivePointSelected()
{
  for(int i = 0;i < m_NumberOfPoints;i++)
  {
    if(m_MeasurementsPoints.at(i)->GetIsPointSelected())
    {
      m_PointSet->SetSelectInfo(i,true);
    }
    else
    {
      m_PointSet->SetSelectInfo(i,false);
    }
  }
}

void MeasurementsPoints::UpdatePointSet()
{
  for( int i=0; i < m_NumberOfPoints; i++)
  {
    mitk::Point3D p = m_MeasurementsPoints.at(i)->GetPoint3D();
    m_PointSet->InsertPoint(i, p , 0);
    m_Points->InsertPoint(i, p[0], p[1], p[2]);
  }
  m_PointSet->Update();
  m_PointSetNode->Update();

}

void MeasurementsPoints::FillPointNames()
{
  for(int i=0; i < m_NumberOfPoints; i++)
  {
    m_PointNames->InsertValue(i, m_MeasurementsPoints.at(i)->GetPointName());
  }
}
mitk::MeasurementsPoint* MeasurementsPoints::GetMPointAtPosition(int i)
{
  if(i < 0 || i >= m_NumberOfPoints) return NULL;

  return m_MeasurementsPoints.at(i);
}




void MeasurementsPoints::UpdateOutputInformation()
{
  // Use methode of BaseData; override in sub-classes
  Superclass::UpdateOutputInformation();
}

void MeasurementsPoints::SetRequestedRegionToLargestPossibleRegion()
{
  // does not apply
}

bool MeasurementsPoints::RequestedRegionIsOutsideOfTheBufferedRegion()
{
  // does not apply
  return false;
}

bool MeasurementsPoints::VerifyRequestedRegion()
{
  // does not apply
  return true;
}

void MeasurementsPoints::SetRequestedRegion(const itk::DataObject * )
{
  // does not apply
}
}