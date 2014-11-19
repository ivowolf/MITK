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

#ifndef MITK_MeasurementsPoint_H
#define MITK_MeasurementsPoint_H

#include <itkObject.h>
#include <mitkDataNode.h>
#include <mitkVector.h>
#include <MitkEndoscopyCalibrationExports.h>
namespace mitk
{
  /*!
  \brief mitk::MeasurementsPoint - This class provides get/set function for storing a specific measurement point

  \details This class is used to store the 3D coordinate of the point as welll as additional information such as
  the id and name of the point, the description, the position in the 2D scheme (QmitkHeartMeasurementsSchemeWidget)
  and wheter it is currently selected in the scheme.


  */
  class MITK_ENDOSCOPYCALIBRATION_EXPORT MeasurementsPoint : public itk::Object
  {
  public:

    MeasurementsPoint(unsigned int id, std::string name, std::string description, mitk::Point2D positionInScheme, bool selected = false);
    virtual ~MeasurementsPoint();

    ///@{
    /**
    * \brief Getter and setter for member variables
    */
    itkGetMacro(IsPointSelected, bool);
    itkSetMacro(IsPointSelected, bool);

    itkGetMacro(PointId, unsigned int);
    itkSetMacro(PointId, unsigned int);

    itkGetMacro(PointName, std::string );
    itkSetMacro(PointName, std::string);

    itkGetMacro(PointDescription, std::string );
    itkSetMacro(PointDescription, std::string);

    itkGetMacro(PositionInScheme, mitk::Point2D);
    itkSetMacro(PositionInScheme, mitk::Point2D);

    itkGetMacro(IsPointLogged, bool);
    itkSetMacro(IsPointLogged, bool);

    itkGetMacro(Point3D, mitk::Point3D);
    itkSetMacro(Point3D, mitk::Point3D);

    ///@}

  private:

    /**
    * \brief unique point id starting at 1
    */
    unsigned int m_PointId;

    /**
    * \brief unique point name
    */
    std::string m_PointName;

    /**
    * \brief description that is shown in the QmitkHeartMeasurementsSchemeWidget::m_String_Description
    */
    std::string m_PointDescription;

    /**
    * \brief position of point in the 2D QmitkHeartMeasurementsSchemeWidget
    */
    mitk::Point2D m_PositionInScheme;

    /**
    * \brief stores information whether point is already set
    */
    bool m_IsPointLogged;

    /**
    * \brief stores information whether point is currently selected
    */
    bool m_IsPointSelected;

    /**
    * \brief 3D point coordinates
    */
    mitk::Point3D m_Point3D;

  };
}

# endif