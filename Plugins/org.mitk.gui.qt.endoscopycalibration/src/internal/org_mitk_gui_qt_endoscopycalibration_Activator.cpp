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


#include "org_mitk_gui_qt_endoscopycalibration_Activator.h"

#include <QtPlugin>

#include "QmitkEndoscopyCalibrationView.h"
#include <usModuleInitialization.h>
namespace mitk {

ctkPluginContext* org_mitk_gui_qt_endoscopycalibration_Activator::m_Context = 0;

void org_mitk_gui_qt_endoscopycalibration_Activator::start(ctkPluginContext* context)
{
  m_Context = context;
  BERRY_REGISTER_EXTENSION_CLASS(QmitkEndoscopyCalibrationView, context)
}

ctkPluginContext* org_mitk_gui_qt_endoscopycalibration_Activator::GetContext()
{
  return m_Context;

}
void org_mitk_gui_qt_endoscopycalibration_Activator::stop(ctkPluginContext* context)
{
  m_Context = 0;
  Q_UNUSED(context)
}

}

Q_EXPORT_PLUGIN2(org_mitk_gui_qt_endoscopycalibration, mitk::org_mitk_gui_qt_endoscopycalibration_Activator)
US_INITIALIZE_MODULE("Endoscopy Calibration", "liborg_mitk_gui_qt_endoscopycalibration")
