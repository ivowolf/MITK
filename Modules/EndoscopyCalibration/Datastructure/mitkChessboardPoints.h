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

#ifndef MITK_ChessboardPoints_H
#define MITK_ChessboardPoints_H


#include "mitkMeasurementsPoints.h"
#include <utility>
#include <MitkEndoscopyCalibrationExports.h>

namespace mitk
{
  /*!
  \brief mitk::ChessboardPoints - This class holds the specific structure of
  chessboard points measurement procedure for camera calibration

  \details A specific number of points are logged to calculate rotation and transformation of the camera and get a
  relation between camera coordinate system and polaris coordinate system.


  */
  class MITK_ENDOSCOPYCALIBRATION_EXPORT ChessboardPoints : public MeasurementsPoints
  {
  public:
    mitkClassMacro( ChessboardPoints, MeasurementsPoints );
    itkNewMacro( Self );
    enum ChessBoardPoints {
      LEFT_UPPER = 0,
      RIGHT_UPPER = 1,
      LEFT_LOWER = 2,
      RIGHT_LOWER = 3,
      //MIDDLE_POINT = 4
    };
    /**
    *   \brief Defines and adds all Measurement Points
    */
    void defineAllMeasurementPoints();

  protected:
    ChessboardPoints();
    virtual ~ChessboardPoints();
  };
}

# endif