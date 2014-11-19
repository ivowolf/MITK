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

#ifndef QMITKCHESSBOARDSCHEMEWIDGET_H
#define QMITKCHESSBOARDSCHEMEWIDGET_H

#include <mitkMeasurementsPoints.h>
#include <mitkMeasurementsPoint.h>
#include <QWidget>
#include <QPainter>

/*!
\brief QmitkChessboardSchemeWidget - QT Widget for providing a measurement scheme, that shows which
point is currently selected or already logged.

\ingroup  org_mbi_gui_qt_heartmeasurements_internal
*/
class QmitkChessboardSchemeWidget : public QWidget
{
  Q_OBJECT

public:
  QmitkChessboardSchemeWidget( QWidget * parent=0);

  virtual ~QmitkChessboardSchemeWidget();

  /**
  * \brief This function is called by QT. It draw the image in the background and calls DrawPoint().
  */
  virtual void paintEvent( QPaintEvent* e );

  /**
  * \brief Sets m_SelectedPoint and updates the widget
  */
  void SetSelectedPoint( int i );

   /**
  * \brief Sets m_MeasurementPoints, which is the collection of measurement points associated to this scheme
  */
  void InitializeMeasurementPoints(mitk::MeasurementsPoints* mPoints);

signals:
  /**
  * \brief QT signal function that passes the index of the selected point to the QT view
  */
  void PointSelected( int i );
  /**
  * \brief QT signal function that passes the Measurementspoint index to the QT view
  */
  void SelectedPoint(mitk::MeasurementsPoint* m);

protected:
  /**
  * \brief called by QT if a mouse button is pressed. The mouse coordinates are evaluated and according to their
  * position (whether they are inside or outside the circle).
  */
  virtual void mousePressEvent( QMouseEvent *e );

  /**
  * \brief Draw 2D circles in the scheme. Called by paintEvent()
  * \param i  MeasurementsPoint index
  * \param logged  Circle has green color if point has already been set
  * \param selected  Circle has red color if point is selected
  */
  void DrawPoint( int i, bool logged, bool selected);

private:

  /**
  * \brief Used for drawing the circles
  */
  QPainter m_Painter;

  /**
  * \brief store the index of the currently selected points
  */
  int m_SelectedPoint;

   /**
  * \brief Each QmitkHeartMeasurementsSchemeWidget must have a MeasurementsPoints procedure
  */
  mitk::MeasurementsPoints * m_MeasurementPoints;
};

#endif // QMITKCHESSBOARDSCHEMEWIDGET_H