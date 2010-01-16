/*=========================================================================

 Program:   BlueBerry Platform
 Language:  C++
 Date:      $Date$
 Version:   $Revision: 17020 $

 Copyright (c) German Cancer Research Center, Division of Medical and
 Biological Informatics. All rights reserved.
 See MITKCopyright.txt or http://www.mitk.org/copyright.html for details.

 This software is distributed WITHOUT ANY WARRANTY; without even
 the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the above copyright notices for more information.

 =========================================================================*/

#include "mitkCoreExtActivator.h"

#include <mitkCoreExtObjectFactory.h>

namespace mitk
{

void
CoreExtActivator::Start(berry::IBundleContext::Pointer /*context*/)
{
  RegisterCoreExtObjectFactory();
}

}
