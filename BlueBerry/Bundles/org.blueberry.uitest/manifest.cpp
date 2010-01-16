/*=========================================================================
 
 Program:   BlueBerry Platform
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

#include "Poco/ClassLibrary.h"

#include <berryIApplication.h>
#include "src/berryUITestApplication.h"

#include <berryIPerspectiveFactory.h>
#include "src/util/berryEmptyPerspective.h"

POCO_BEGIN_NAMED_MANIFEST(berryIApplication, berry::IApplication)
  POCO_EXPORT_CLASS(berry::UITestApplication)
POCO_END_MANIFEST

POCO_BEGIN_NAMED_MANIFEST(berryIPerspectiveFactory, berry::IPerspectiveFactory)
  POCO_EXPORT_CLASS(berry::EmptyPerspective)
POCO_END_MANIFEST
