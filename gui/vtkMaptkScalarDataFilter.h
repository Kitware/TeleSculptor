// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef vtkMaptkScalarDataFilter_h
#define vtkMaptkScalarDataFilter_h

#include "vtkDataSetAlgorithm.h"

class vtkMaptkScalarDataFilter : public vtkDataSetAlgorithm
{
public:
  static vtkMaptkScalarDataFilter *New();
  vtkTypeMacro(vtkMaptkScalarDataFilter,vtkDataSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  // Description:
  // Get/Set the point data array name to set as active scalars on the output.
  vtkSetStringMacro(ScalarArrayName);
  vtkGetStringMacro(ScalarArrayName);

protected:
  vtkMaptkScalarDataFilter();
  ~vtkMaptkScalarDataFilter() override;

  int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector) override;

  char* ScalarArrayName;

private:
  vtkMaptkScalarDataFilter(vtkMaptkScalarDataFilter const&) = delete;
  void operator=(vtkMaptkScalarDataFilter const&) = delete;
};

#endif
