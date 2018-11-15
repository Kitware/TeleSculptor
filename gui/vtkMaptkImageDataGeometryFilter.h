/*ckwg +29
* Copyright 2016-2018 by Kitware, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
*  * Neither the name Kitware, Inc. nor the names of any contributors may be
*    used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
