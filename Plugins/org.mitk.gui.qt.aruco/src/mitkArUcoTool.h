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


#ifndef MITKARUCOTOOL_H_HEADER_INCLUDED_
#define MITKARUCOTOOL_H_HEADER_INCLUDED_

#include <org_mitk_gui_qt_aruco_Export.h>

#include <mitkInternalTrackingTool.h>
//#include "mitkTrackingTypes.h"

namespace mitk {
    //##Documentation
    //## \brief Implementation of a ArUco tool
    //##
    //##
    //## \ingroup IGT

    class ARUCO_EXPORT ArUcoTool : public InternalTrackingTool
    {
    public:
      mitkClassMacro(ArUcoTool, InternalTrackingTool);
      itkNewMacro(Self);
    protected:
      ArUcoTool();
      virtual ~ArUcoTool();
    };
} // namespace mitk

#endif /* MITKARUCOTOOL_H_HEADER_INCLUDED_ */
