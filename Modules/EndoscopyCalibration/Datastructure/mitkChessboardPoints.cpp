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

#include "mitkChessboardPoints.h"
#include <sstream>

namespace mitk
{
 ChessboardPoints::ChessboardPoints()
  {
    // zero at the beginning, since points are added on-the-fly
    // incremented in the process of measuring points
    m_NumberOfPoints = 4;
    m_MeasurementsPoints.resize(m_NumberOfPoints);
    this->defineAllMeasurementPoints();
    m_NameOfSchemeFile = ":/QmitkEndoscopyCalibrationView/chessboard.svg";

    m_PointSetNode->SetName("ChessboardPoints");
    m_PointSetNode->SetColor(0.0f, 0.0f, 1.0f);
    //m_InterpolatedConnectionNode->SetName("Spline_AnnulusPoints");
    m_Points->SetNumberOfPoints(m_NumberOfPoints);
  }

  ChessboardPoints::~ChessboardPoints()
  {
  }


  void ChessboardPoints::defineAllMeasurementPoints()
  {
     // Position of Point in Scheme
    mitk::ScalarType point00[] = { 1.005, 0.215, 0.0 }; //1
    mitk::ScalarType point01[] = { 0.025, 0.215, 0.0 }; // 2
    mitk::ScalarType point02[] = { 1.005, -0.485, 0.0 }; // 3
    mitk::ScalarType point03[] = { 0.025, -0.485, 0.0 }; // 4
    //mitk::ScalarType point04[] = { 0.640, -0.340, 0.0 }; // 5
     m_MeasurementsPoints.at(LEFT_UPPER)
      = new mitk::MeasurementsPoint(1, "First Point", "The First Point is the left upper Point in Chessboard Scheme", mitk::Point2D( point00 ));

    m_MeasurementsPoints.at(RIGHT_UPPER)
      = new mitk::MeasurementsPoint(2, "Second Point", "The Second Point is the right upper Point in Chessboard Scheme", mitk::Point2D( point01 ));

    m_MeasurementsPoints.at(LEFT_LOWER)
      = new mitk::MeasurementsPoint(3, "Third Point", "The Third Point is the left lower Point in Chessboard Scheme", mitk::Point2D( point02 ));

    m_MeasurementsPoints.at(RIGHT_LOWER)
      = new mitk::MeasurementsPoint(4, "Fourth Point", "The Fourth Point is the right lower Point in Chessboard Scheme", mitk::Point2D( point03 ));

   // m_MeasurementsPoints.at(MIDDLE_POINT)
    //= new mitk::MeasurementsPoint(5, "Fifth Point", "The Fourth Point is near to the middle", mitk::Point2D( point04 ));

    FillPointNames();
  }

} // namespace