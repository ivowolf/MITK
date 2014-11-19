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

#include "mitkMeasurementsPoint.h"

namespace mitk
{
 MeasurementsPoint::MeasurementsPoint(unsigned int id, std::string name, std::string description, mitk::Point2D positionInScheme, bool selected)
    : m_PointId(id),
    m_PointName(name),
    m_PointDescription(description),
    m_PositionInScheme(positionInScheme),
    m_IsPointLogged(false),
    m_IsPointSelected(selected)
  {
    // value per default in the beginning
    mitk::ScalarType point[] = { 0, 0, 0 };
    m_Point3D = mitk::Point3D(point);
  }

  MeasurementsPoint::~MeasurementsPoint()
  {
  }
} // namespace