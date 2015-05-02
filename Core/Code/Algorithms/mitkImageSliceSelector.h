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


#ifndef IMAGESLICESELECTOR_H_HEADER_INCLUDED_C1E4BE7B
#define IMAGESLICESELECTOR_H_HEADER_INCLUDED_C1E4BE7B

#include <MitkCoreExports.h>
#include "mitkSubImageSelector.h"

namespace mitk {

//##Documentation
//## @brief Provides access to a slice of the input image
//##
//## If the input is generated by a ProcessObject, only the required data is
//## requested.
//## @ingroup Process
class MITK_CORE_EXPORT ImageSliceSelector : public SubImageSelector
{
  public:

  mitkClassMacro(ImageSliceSelector,SubImageSelector);

  itkFactorylessNewMacro(Self)
  itkCloneMacro(Self)

  itkGetConstMacro(SliceNr,int);
  itkSetMacro(SliceNr,int);

  itkGetConstMacro(TimeNr,int);
  itkSetMacro(TimeNr,int);

  itkGetConstMacro(ChannelNr,int);
  itkSetMacro(ChannelNr,int);

protected:
  virtual void GenerateOutputInformation();

  virtual void GenerateInputRequestedRegion();

  virtual void GenerateData();

  ImageSliceSelector();

  virtual ~ImageSliceSelector();

  int m_SliceNr;

  int m_TimeNr;

  int m_ChannelNr;

};

} // namespace mitk



#endif /* IMAGESLICESELECTOR_H_HEADER_INCLUDED_C1E4BE7B */