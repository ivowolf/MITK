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

#include "berryICommandCategoryListener.h"

#include "berryCommandCategory.h"
#include "berryCommandCategoryEvent.h"

namespace berry {

void
ICommandCategoryListener::Events
::AddListener(ICommandCategoryListener::Pointer l)
{
  if (l.IsNull()) return;

  categoryChanged += Delegate(l.GetPointer(), &ICommandCategoryListener::CategoryChanged);
}

void
ICommandCategoryListener::Events
::RemoveListener(ICommandCategoryListener::Pointer l)
{
  if (l.IsNull()) return;

  categoryChanged -= Delegate(l.GetPointer(), &ICommandCategoryListener::CategoryChanged);
}

}


