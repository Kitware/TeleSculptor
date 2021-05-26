// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "vtkMaptkScalarDataFilter.h"

#include "vtkDataSet.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"

vtkStandardNewMacro(vtkMaptkScalarDataFilter);

//----------------------------------------------------------------------------
vtkMaptkScalarDataFilter::vtkMaptkScalarDataFilter()
{
  this->ScalarArrayName = nullptr;
}

//----------------------------------------------------------------------------
vtkMaptkScalarDataFilter::~vtkMaptkScalarDataFilter()
{
  this->SetScalarArrayName(nullptr);
}
//----------------------------------------------------------------------------
int vtkMaptkScalarDataFilter::RequestData(
  vtkInformation*,
  vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  vtkInformation* info = outputVector->GetInformationObject(0);
  vtkDataSet *output = vtkDataSet::SafeDownCast(
    info->Get(vtkDataObject::DATA_OBJECT()));

  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkDataSet *input = vtkDataSet::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkDebugMacro(<< "Setting scalar array");

  output->ShallowCopy(input);

  if (this->ScalarArrayName)
  {
    output->GetPointData()->SetActiveScalars(this->ScalarArrayName);
  }

  return 1;
}

//----------------------------------------------------------------------------
void vtkMaptkScalarDataFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
