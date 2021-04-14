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
#include "vtkMaptkImageDataGeometryFilter.h"

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkExecutive.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>


#include <map>

vtkStandardNewMacro(vtkMaptkImageDataGeometryFilter);

//-----------------------------------------------------------------------------
class vtkMaptkImageDataGeometryFilter::vtkInternal
{
public:
  struct ConstraintRange
  {
    ConstraintRange()
    {
      this->MinValue = 1;
      this->MaxValue = -1;
    }

    ConstraintRange(double minValue, double maxValue)
    {
      this->MinValue = minValue;
      this->MaxValue = maxValue;
    }

    double MinValue;
    double MaxValue;
  };

  typedef std::map<std::string, ConstraintRange> ConstraintType;

  ConstraintType Constraints;
};

// Construct with initial extent of all the data
vtkMaptkImageDataGeometryFilter::vtkMaptkImageDataGeometryFilter()
  : Internal(new vtkInternal)
{
  this->ThresholdCells = 0;
  this->GenerateTriangleOutput = 0;

  this->UnprojectedPointArrayName = nullptr;
  this->SetUnprojectedPointArrayName("Points");

  this->SetNumberOfOutputPorts(3);
}

//-----------------------------------------------------------------------------
vtkMaptkImageDataGeometryFilter::~vtkMaptkImageDataGeometryFilter()
{
  this->SetUnprojectedPointArrayName(nullptr);
}

//-----------------------------------------------------------------------------
void vtkMaptkImageDataGeometryFilter::SetConstraint(
  const char* arrayName, double minValue, double maxValue)
{
  std::string strArrayName(arrayName);
  vtkInternal::ConstraintType::iterator constraint =
    this->Internal->Constraints.find(strArrayName);
  if (constraint == this->Internal->Constraints.end() ||
      constraint->second.MinValue != minValue ||
      constraint->second.MaxValue != maxValue)
  {
    vtkInternal::ConstraintRange range(minValue, maxValue);
    this->Internal->Constraints[strArrayName] = range;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkMaptkImageDataGeometryFilter::RemoveConstraint(const char* arrayName)
{
  std::string strArrayName(arrayName);
  vtkInternal::ConstraintType::iterator constraint =
    this->Internal->Constraints.find(strArrayName);
  if (constraint != this->Internal->Constraints.end())
  {
    this->Internal->Constraints.erase(strArrayName);
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkMaptkImageDataGeometryFilter::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  // get the input and output
  vtkImageData *input = vtkImageData::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData *output = vtkPolyData::SafeDownCast(
    outInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData* outputUnprojected = vtkPolyData::GetData(outputVector, 1);
  vtkPolyData* outputUnprojectedPolys = vtkPolyData::GetData(outputVector, 2);

  vtkPoints *newPts = nullptr;
  vtkDebugMacro(<< "Extracting structured points geometry");

  // Output points are the full set of image points (and retain all PointData);
  // Control render behavior via cell data
  int dims[3], extents[6];
  double origin[3], spacing[3];
  input->GetDimensions(dims);
  input->GetExtent(extents);
  input->GetOrigin(origin);
  input->GetSpacing(spacing);
  vtkIdType numberOfPoints = input->GetNumberOfPoints();

  if (dims[0] < 2 || dims[1] < 2 || dims[2] != 1)
  {
    vtkErrorMacro(<< "Expecting XY plane but input is " << dims[0] <<
      " by " << dims[1] << " by " << dims[2]);
    return 1;
  }

  newPts = vtkPoints::New();
  output->SetPoints(newPts);
  newPts->FastDelete();
  newPts->SetNumberOfPoints(numberOfPoints);

  // Expect 3D point data for the 2nd output; if not present, use same points
  // as 1st input
  vtkDataArray* inputPoints = input->GetPointData()->GetArray(
    this->UnprojectedPointArrayName);
  if (!inputPoints || inputPoints->GetNumberOfComponents() != 3)
  {
    vtkErrorMacro(<< "Unable to find / use input3D points for 2nd output");
    outputUnprojected->SetPoints(newPts);
    outputUnprojectedPolys->SetPoints(newPts);
  }
  else
  {
    vtkPoints* points3D = vtkPoints::New();
    points3D->SetData(inputPoints);
    outputUnprojected->SetPoints(points3D);
    outputUnprojectedPolys->SetPoints(points3D);
    points3D->FastDelete();
  }

  float* pointsPtr =
    vtkFloatArray::SafeDownCast(newPts->GetData())->GetPointer(0);
  for (int row = extents[2]; row <= extents[3]; ++row)
  {
    float y = origin[1] + row * spacing[1];
    for (int column = extents[0]; column <= extents[1]; ++column)
    {
      *pointsPtr = origin[0] + column * spacing[0];
      *(++pointsPtr) = y;
      *(++pointsPtr) = origin[2];
      ++pointsPtr;
    }
  }

  // Copy the pointData from input to output
  vtkPointData* pointData = input->GetPointData();
  output->GetPointData()->ShallowCopy(pointData);
  outputUnprojected->GetPointData()->ShallowCopy(pointData);
  outputUnprojectedPolys->GetPointData()->ShallowCopy(pointData);

  struct ConstraintType
  {
    vtkDataArray* Array;
    double MinValue;
    double MaxValue;
  };

  std::vector<ConstraintType> constraints;

  // Check any / all constraints
  bool completelyFiltered = false;
  for(auto const& constraint : this->Internal->Constraints)
  {
    vtkDataArray* dataArray = pointData->GetArray(constraint.first.c_str());
    if (dataArray)
    {
      double* range = dataArray->GetRange();
      if (constraint.second.MinValue <= range[0] &&
        constraint.second.MaxValue >= range[1])
      {
        // Doesn't result in any filtering, so skip this filter
        continue;
      }
      else if (constraint.second.MinValue > range[1] ||
        constraint.second.MaxValue < range[0])
      {
        completelyFiltered = true;
        break;
      }

      // add this filter to our filter list
      ConstraintType newConstraint;
      newConstraint.Array = dataArray;
      newConstraint.MinValue = constraint.second.MinValue;
      newConstraint.MaxValue = constraint.second.MaxValue;
      constraints.emplace_back(newConstraint);
    }
  }

  if (completelyFiltered)
  {
    return 1;
  }

  // Add vertices for each point that passes filtering, or all if not
  // threshold cells
  vtkCellArray* newVerts = vtkCellArray::New();
  newVerts->Allocate(numberOfPoints);
  output->SetVerts(newVerts);
  outputUnprojected->SetVerts(newVerts);
  newVerts->FastDelete();

  vtkIdType* validPoints = new vtkIdType[numberOfPoints];
  vtkIdType* currentPt = validPoints;
  // Check each constrainst
  for (vtkIdType i = 0; i < numberOfPoints; ++i)
  {
    if (this->ThresholdCells && constraints.size())
    {
      bool skipCell = false;
      for(auto const constraint : constraints)
      {
        double value = constraint.Array->GetTuple1(i);
        if (value < constraint.MinValue || value > constraint.MaxValue)
        {
          skipCell = true;
          break;
        }
      }

      if (skipCell)
      {
        *currentPt++ = -1;
        continue;
      }
    }

    *currentPt++ = i;
    newVerts->InsertNextCell(1, &i);
  }

  if (this->GenerateTriangleOutput)
  {
    // Add triangles according to the validPoints; all 3 points making up a
    // triangle must be valid
    vtkCellArray* newPolys = vtkCellArray::New();
    int numberOfRowsMinus1 = extents[3] - extents[2];
    int numberOfColumnsMinus1 = extents[1] - extents[0];
    newPolys->Allocate(2 * 4 * numberOfRowsMinus1 * numberOfColumnsMinus1);
    outputUnprojectedPolys->SetPolys(newPolys);
    newPolys->FastDelete();

    // Setup pointers to two first rows of validPoints
    vtkIdType* thisRow = validPoints;
    vtkIdType* nextRow = validPoints + (numberOfColumnsMinus1 + 1);
    vtkIdType triIds[3];
    for (int i = 0; i < numberOfRowsMinus1; ++i, ++thisRow, ++nextRow)
    {
      for (int j = 0; j < numberOfColumnsMinus1; ++j, ++thisRow, ++nextRow)
      {
        // if all 4 points for this quad are valid, add two triangles; if 3 valid
        // points add a single triangle, otherwise add no triangles
        if (*thisRow != -1 && *(thisRow + 1) != -1 &&
          *nextRow != -1 && *(nextRow + 1) != -1)
        {
          triIds[0] = *thisRow;
          triIds[1] = *(thisRow + 1);
          triIds[2] = *(nextRow + 1);
          newPolys->InsertNextCell(3, triIds);
          triIds[1] = triIds[2];
          triIds[2] = *nextRow;
          newPolys->InsertNextCell(3, triIds);
        }
        else if (*thisRow != -1 && *(thisRow + 1) != -1 && *nextRow != -1)
        {
          triIds[0] = *thisRow;
          triIds[1] = *(thisRow + 1);
          triIds[2] = *nextRow;
          newPolys->InsertNextCell(3, triIds);
        }
        else if (*thisRow != -1 && *(thisRow + 1) != -1 && *(nextRow + 1) != -1)
        {
          triIds[0] = *thisRow;
          triIds[1] = *(thisRow + 1);
          triIds[2] = *(nextRow + 1);
          newPolys->InsertNextCell(3, triIds);
        }
        else if (*thisRow != -1 && *nextRow != -1 && *(nextRow + 1) != -1)
        {
          triIds[0] = *thisRow;
          triIds[1] = *(nextRow + 1);
          triIds[2] = *nextRow;
          newPolys->InsertNextCell(3, triIds);
        }
        else if (*(thisRow + 1) != -1 && *nextRow != -1 && *(nextRow + 1) != -1)
        {
          triIds[0] = *(thisRow + 1);
          triIds[1] = *(nextRow + 1);
          triIds[2] = *nextRow;
          newPolys->InsertNextCell(3, triIds);
        }
      }
    }
  }
  else
  {
    outputUnprojectedPolys->SetVerts(newVerts);
  }
  return 1;
}

int vtkMaptkImageDataGeometryFilter::FillInputPortInformation(int, vtkInformation *info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkImageData");
  return 1;
}

void vtkMaptkImageDataGeometryFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "ThresholdCells " << this->ThresholdCells << "\n";
}
