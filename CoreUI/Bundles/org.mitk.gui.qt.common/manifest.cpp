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

#include <Poco/ClassLibrary.h>

#include <berryIEditorPart.h>
#include <berryIBundleActivator.h>

#include "src/QmitkStdMultiWidgetEditor.h"
#include "src/internal/QmitkCommonActivator.h"
#include "src/QmitkGeneralPreferencePage.h"

POCO_BEGIN_NAMED_MANIFEST(berryIEditorPart, berry::IEditorPart)
  POCO_EXPORT_CLASS(QmitkStdMultiWidgetEditor)
POCO_END_MANIFEST

POCO_BEGIN_MANIFEST(berry::IBundleActivator)
  POCO_EXPORT_CLASS(QmitkCommonActivator)
POCO_END_MANIFEST

POCO_BEGIN_NAMED_MANIFEST(berryIPreferencePage, berry::IPreferencePage)
  POCO_EXPORT_CLASS(QmitkGeneralPreferencePage)
POCO_END_MANIFEST
