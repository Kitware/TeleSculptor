// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef vtkMaptkImageDataGeometryFilter_h
#define vtkMaptkImageDataGeometryFilter_h

#include "vtkPolyDataAlgorithm.h"

#include <memory>

class vtkMaptkImageDataGeometryFilter : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkMaptkImageDataGeometryFilter,vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  // Description:
  // Construct with initial extent of all the data
  static vtkMaptkImageDataGeometryFilter *New();

  void SetConstraint(const char* arrayName, double minValue, double maxValue);
  void RemoveConstraint(const char* arrayName);

  // Description:
  // Set ThresholdCells to true if you wish to skip any voxel/pixels which have scalar
  // values that don't satisfy specified constraints. Default is 0 (off)
  vtkSetMacro(ThresholdCells,int);
  vtkGetMacro(ThresholdCells,int);
  vtkBooleanMacro(ThresholdCells,int);

  // Description:
  // Set GenerateTriangleOutput to true if the 3rd output should be filled with
  // triangles (polygons) instead of vertices. If false, this output is the
  // same as the 2nd output. Default is 0 (off).
  vtkSetMacro(GenerateTriangleOutput, int);
  vtkGetMacro(GenerateTriangleOutput, int);
  vtkBooleanMacro(GenerateTriangleOutput, int);

  // Description:
  // Get/Set the unprojected point array name.  Default is "PointsWithDesc".
  vtkSetStringMacro(UnprojectedPointArrayName);
  vtkGetStringMacro(UnprojectedPointArrayName);

protected:
  vtkMaptkImageDataGeometryFilter();
  ~vtkMaptkImageDataGeometryFilter() override;

  int RequestData(vtkInformation*,
                  vtkInformationVector**,
                  vtkInformationVector*) override;
  int FillInputPortInformation(int port, vtkInformation* info) override;

  int    ThresholdCells;
  int    GenerateTriangleOutput;

private:
  vtkMaptkImageDataGeometryFilter(vtkMaptkImageDataGeometryFilter const&) = delete;
  void operator=(vtkMaptkImageDataGeometryFilter const&) = delete;

  char* UnprojectedPointArrayName;

  class vtkInternal;
  vtkInternal* Internal;
};

#endif
