/*ckwg +29
* Copyright 2015 by Kitware, Inc.
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
*  * Neither name of Kitware, Inc. nor the names of any contributors may be used
*    to endorse or promote products derived from this software without specific
*    prior written permission.
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

#include "vtkMaptkFeatureTrackRepresentation.h"

#include <vital/vital_foreach.h>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>

#include <map>

vtkStandardNewMacro(vtkMaptkFeatureTrackRepresentation);

typedef vtkMaptkFeatureTrackRepresentation::TrailStyleEnum TrailStyleEnum;

//-----------------------------------------------------------------------------
class vtkMaptkFeatureTrackRepresentation::vtkInternal
{
public:
  void UpdateActivePoints(unsigned activeFrame);
  void UpdateTrails(unsigned activeFrame, unsigned trailLength,
                    TrailStyleEnum style);

  vtkNew<vtkPoints> Points;

  vtkNew<vtkCellArray> PointsCells;
  vtkNew<vtkCellArray> TrailsCells;

  vtkNew<vtkPolyData> PointsPolyData;
  vtkNew<vtkPolyData> TrailsPolyData;

  typedef std::map<unsigned, vtkIdType> TrackType;
  typedef std::map<unsigned, TrackType> TrackMapType;

  TrackMapType Tracks;
};

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::vtkInternal::UpdateActivePoints(
  unsigned activeFrame)
{
  this->PointsCells->Reset();

  VITAL_FOREACH(auto const& t, this->Tracks)
  {
    auto const& track = t.second;
    auto const fi = track.find(activeFrame);
    if (fi != track.cend())
    {
      this->PointsCells->InsertNextCell(1);
      this->PointsCells->InsertCellPoint(fi->second);
    }
  }

  this->PointsPolyData->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::vtkInternal::UpdateTrails(
  unsigned activeFrame, unsigned trailLength, TrailStyleEnum style)
{
  this->TrailsCells->Reset();

  auto const symmetric =
    (style == vtkMaptkFeatureTrackRepresentation::Symmetric);

  auto const minFrame =
    (trailLength > activeFrame ? 0 : activeFrame - trailLength);
  auto const maxFrame = (symmetric ? activeFrame + trailLength : activeFrame);

  std::vector<vtkIdType> points;

  VITAL_FOREACH (auto const& ti, this->Tracks)
  {
    auto const& track = ti.second;
    if (track.cbegin()->first > activeFrame ||
        (--track.cend())->first < activeFrame)
    {
      // Skip tracks that are not active on the active frame
      continue;
    }

    // Build list of relevant points
    points.clear();

    auto const fe = track.upper_bound(maxFrame);
    for (auto fi = track.lower_bound(minFrame); fi != fe; ++fi)
    {
      points.push_back(fi->second);
    }

    // Create cell for trail (only if trail is non-empty)
    auto const n = static_cast<vtkIdType>(points.size());
    if (n > 1)
    {
      this->TrailsCells->InsertNextCell(points.size(), points.data());
    }
  }

  this->TrailsPolyData->Modified();
}

//-----------------------------------------------------------------------------
vtkMaptkFeatureTrackRepresentation::vtkMaptkFeatureTrackRepresentation()
  : Internal(new vtkInternal)
{
  this->TrailLength = 2;

  this->ActiveFrame = 0;

  // Set up actors and data
  vtkNew<vtkPolyDataMapper> pointsMapper;

  this->Internal->PointsPolyData->SetPoints(
    this->Internal->Points.GetPointer());
  this->Internal->PointsPolyData->SetVerts(
    this->Internal->PointsCells.GetPointer());

  pointsMapper->SetInputData(this->Internal->PointsPolyData.GetPointer());

  vtkNew<vtkPolyDataMapper> trailsMapper;

  this->Internal->TrailsPolyData->SetPoints(
    this->Internal->Points.GetPointer());
  this->Internal->TrailsPolyData->SetLines(
    this->Internal->TrailsCells.GetPointer());

  trailsMapper->SetInputData(this->Internal->TrailsPolyData.GetPointer());

  this->ActivePointsActor = vtkSmartPointer<vtkActor>::New();
  this->ActivePointsActor->SetMapper(pointsMapper.GetPointer());

  this->TrailsActor = vtkSmartPointer<vtkActor>::New();
  this->TrailsActor->SetMapper(trailsMapper.GetPointer());
}

//-----------------------------------------------------------------------------
vtkMaptkFeatureTrackRepresentation::~vtkMaptkFeatureTrackRepresentation()
{
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::AddTrackPoint(
  unsigned trackId, unsigned frameId, double x, double y)
{
  auto const id = this->Internal->Points->InsertNextPoint(x, y, 0.0);
  this->Internal->Tracks[trackId][frameId] = id;
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::SetActiveFrame(unsigned frame)
{
  if (this->ActiveFrame == frame)
  {
    return;
  }

  this->ActiveFrame = frame;
  this->Update();
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::SetTrailLength(unsigned length)
{
  if (this->TrailLength == length)
  {
    return;
  }

  this->TrailLength = length;
  this->Internal->UpdateTrails(this->ActiveFrame,
                               this->TrailLength, this->TrailStyle);
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::SetTrailStyle(TrailStyleEnum style)
{
  if (this->TrailStyle == style)
  {
    return;
  }

  this->TrailStyle = style;
  this->Internal->UpdateTrails(this->ActiveFrame,
                               this->TrailLength, this->TrailStyle);
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::Update()
{
  this->Internal->UpdateActivePoints(this->ActiveFrame);
  this->Internal->UpdateTrails(this->ActiveFrame,
                               this->TrailLength, this->TrailStyle);
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::PrintSelf(
  ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Number Of Points: "
     << this->Internal->Points->GetNumberOfPoints() << endl;
  os << indent << "ActiveFrame: "
     << this->ActiveFrame << endl;
  os << indent << "TrailLength: "
     << this->TrailLength << endl;
}
