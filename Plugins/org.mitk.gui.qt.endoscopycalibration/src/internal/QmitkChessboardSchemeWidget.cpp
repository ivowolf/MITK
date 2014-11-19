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

#include "QmitkChessboardSchemeWidget.h"
#include <QMouseEvent>

QmitkChessboardSchemeWidget::QmitkChessboardSchemeWidget(QWidget * parent)
: QWidget(parent),
m_Painter( this ),
m_SelectedPoint( -1 )
{
}

QmitkChessboardSchemeWidget::~QmitkChessboardSchemeWidget()
{
}

void QmitkChessboardSchemeWidget::SetSelectedPoint( int i )
  {
    MITK_INFO <<"Selected Point "<< i <<std::endl;
    m_SelectedPoint = i;
    QmitkChessboardSchemeWidget::update();
  }

void QmitkChessboardSchemeWidget::InitializeMeasurementPoints(mitk::MeasurementsPoints * mPoints)
{
  m_MeasurementPoints = mPoints;
}

// only called by paintevent
void QmitkChessboardSchemeWidget::DrawPoint( int i, bool logged, bool selected)
{
  m_Painter.setBrush( QBrush( Qt::lightGray ) );

  if (logged)
  {
    m_Painter.setBrush( QBrush( Qt::green ) );
  }
  if(selected)
  {
    m_Painter.setBrush( QBrush( Qt::red ) );
  }

   m_Painter.drawEllipse(
    (int)(this->width() / 2.0 - (m_MeasurementPoints->GetMPointAtPosition(i)->GetPositionInScheme()[0] - 0.5) * 410.0), // - 8.0),
    (int)(this->height() / 2.0 - (m_MeasurementPoints->GetMPointAtPosition(i)->GetPositionInScheme()[1] + 0.15) * 410.0), // - 8.0),
    12, 12 );
}

void QmitkChessboardSchemeWidget::paintEvent( QPaintEvent * /*paintEvent*/ )
{
  m_Painter.begin(this);

  // paint scheme into background
  m_Painter.drawPixmap(0, 0, QPixmap(QString::fromStdString(m_MeasurementPoints->GetNameOfSchemeFile())).scaled(size()));

  m_Painter.setPen( Qt::black );

  QBrush brush( Qt::green);
  m_Painter.setBrush( brush );

  int i;
  for ( i = 0; i < m_MeasurementPoints->GetNumberOfPoints(); ++i )
  {
    this->DrawPoint( i, m_MeasurementPoints->GetMPointAtPosition(i)->GetIsPointLogged(), i == m_SelectedPoint);
  }
  m_Painter.end();
}

void QmitkChessboardSchemeWidget::mousePressEvent( QMouseEvent *mouseEvent )
{
  const int x = mouseEvent->x();
  const int y = mouseEvent->y();

  int i;
  // Set all Measurement points unselected
  for ( i = 0; i < m_MeasurementPoints->GetNumberOfPoints(); ++i )
  {
    m_MeasurementPoints->GetMPointAtPosition(i)->SetIsPointSelected(false);
  }
  for ( i = 0; i < m_MeasurementPoints->GetNumberOfPoints(); ++i )
  {
    int dx = x - (int)(width() / 2.0 - (m_MeasurementPoints->GetMPointAtPosition(i)->GetPositionInScheme()[0] - 0.5) * 410.0);
    int dy = y - (int)(height() / 2.0 - (m_MeasurementPoints->GetMPointAtPosition(i)->GetPositionInScheme()[1] + 0.15) * 410.0);

    int squaredDistance = dx*dx + dy*dy;

    if ( squaredDistance < 100 )
    {
      // Highlight selected Measurement Point on Renderwindow
      m_MeasurementPoints->GetMPointAtPosition(i)->SetIsPointSelected(true);
      m_MeasurementPoints->SetActivePointSelected();
      m_MeasurementPoints->Update();
      emit PointSelected(i);
      emit SelectedPoint(m_MeasurementPoints->GetMPointAtPosition(i));
      SetSelectedPoint(i);
      MITK_INFO <<"Selected Point Index on Scheme: " << i+1 <<  std::endl;
      QmitkChessboardSchemeWidget::update(); // initiates new paint event
      break;
    }
  }
}