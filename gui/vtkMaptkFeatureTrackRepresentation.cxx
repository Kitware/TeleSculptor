// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "vtkMaptkFeatureTrackRepresentation.h"

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
  void UpdateActivePoints(kwiver::vital::frame_id_t activeFrame);
  void UpdateTrails(kwiver::vital::frame_id_t activeFrame,
                    unsigned trailLength, TrailStyleEnum style);

  vtkNew<vtkPoints> PointsWithDesc;
  vtkNew<vtkPoints> PointsWithoutDesc;

  vtkNew<vtkCellArray> PointsWithDescCells;
  vtkNew<vtkCellArray> PointsWithoutDescCells;

  vtkNew<vtkCellArray> TrailsWithDescCells;
  vtkNew<vtkCellArray> TrailsWithoutDescCells;

  vtkNew<vtkPolyData> PointsWithDescPolyData;
  vtkNew<vtkPolyData> PointsWithoutDescPolyData;
  vtkNew<vtkPolyData> TrailsWithDescPolyData;
  vtkNew<vtkPolyData> TrailsWithoutDescPolyData;

  using TrackType    = std::map<kwiver::vital::frame_id_t, vtkIdType>;
  using TrackMapType = std::map<kwiver::vital::track_id_t, TrackType>;

  TrackMapType TracksWithDesc;
  TrackMapType TracksWithoutDesc;
};

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::vtkInternal::UpdateActivePoints(
  kwiver::vital::frame_id_t activeFrame)
{
  this->PointsWithDescCells->Reset();

  for(auto const& t : this->TracksWithDesc)
  {
    auto const& track = t.second;
    auto const fi = track.find(activeFrame);
    if (fi != track.cend())
    {
      this->PointsWithDescCells->InsertNextCell(1);
      this->PointsWithDescCells->InsertCellPoint(fi->second);
    }
  }

  this->PointsWithDescCells->Modified();
  this->PointsWithDescPolyData->Modified();

  this->PointsWithoutDescCells->Reset();

  for (auto const& t : this->TracksWithoutDesc)
  {
    auto const& track = t.second;
    auto const fi = track.find(activeFrame);
    if (fi != track.cend())
    {
      this->PointsWithoutDescCells->InsertNextCell(1);
      this->PointsWithoutDescCells->InsertCellPoint(fi->second);
    }
  }

  this->PointsWithoutDescCells->Modified();
  this->PointsWithoutDescPolyData->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::vtkInternal::UpdateTrails(
  kwiver::vital::frame_id_t activeFrame, unsigned trailLength,
  TrailStyleEnum style)
{
  this->TrailsWithDescCells->Reset();
  this->TrailsWithoutDescCells->Reset();

  auto const symmetric =
    (style == vtkMaptkFeatureTrackRepresentation::Symmetric);

  auto const minFrame =
    (trailLength > activeFrame ? 0 : activeFrame - trailLength);
  auto const maxFrame = (symmetric ? activeFrame + trailLength : activeFrame);

  std::vector<vtkIdType> points;

  for (auto const& ti : this->TracksWithDesc)
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

    bool active_found = false;

    //put all correspondences in for features that include descriptors
    auto const fe = track.upper_bound(maxFrame);
    for (auto fi = track.lower_bound(minFrame); fi != fe; ++fi)
    {
      points.push_back(fi->second);
      if (fi->first == activeFrame)
      {
        active_found = true;
      }
    }

    // Create cell for trail (only if trail is non-empty)
    auto const n = static_cast<vtkIdType>(points.size());
    if (n > 1 && active_found)
    {
      this->TrailsWithDescCells->InsertNextCell(points.size(), points.data());
    }
  }

  for (auto const& ti : this->TracksWithoutDesc)
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

    bool active_found = false;
    auto const fe = track.upper_bound(maxFrame);
    for (auto fi = track.lower_bound(minFrame); fi != fe; ++fi)
    {
      points.push_back(fi->second);
      if (fi->first == activeFrame)
      {
        active_found = true;
      }
    }

    // Create cell for trail (only if trail is non-empty)
    auto const n = static_cast<vtkIdType>(points.size());
    if (n > 1 && active_found)
    {
      this->TrailsWithoutDescCells->InsertNextCell(points.size(), points.data());
    }
  }

  this->TrailsWithDescCells->Modified();
  this->TrailsWithoutDescCells->Modified();
  this->TrailsWithDescPolyData->Modified();
  this->TrailsWithoutDescPolyData->Modified();
}

//-----------------------------------------------------------------------------
vtkMaptkFeatureTrackRepresentation::vtkMaptkFeatureTrackRepresentation()
  : Internal(new vtkInternal)
{
  this->TrailLength = 2;

  this->ActiveFrame = 0;

  // Set up actors and data
  vtkNew<vtkPolyDataMapper> pointsWithDescMapper;
  vtkNew<vtkPolyDataMapper> pointsWithoutDescMapper;

  this->Internal->PointsWithDescPolyData->SetPoints(
    this->Internal->PointsWithDesc.GetPointer());
  this->Internal->PointsWithDescPolyData->SetVerts(
    this->Internal->PointsWithDescCells.GetPointer());

  this->Internal->PointsWithoutDescPolyData->SetPoints(
    this->Internal->PointsWithoutDesc.GetPointer());
  this->Internal->PointsWithoutDescPolyData->SetVerts(
    this->Internal->PointsWithoutDescCells.GetPointer());

  pointsWithDescMapper->SetInputData(this->Internal->PointsWithDescPolyData.GetPointer());
  pointsWithoutDescMapper->SetInputData(this->Internal->PointsWithoutDescPolyData.GetPointer());

  vtkNew<vtkPolyDataMapper> trailsWithDescMapper;
  vtkNew<vtkPolyDataMapper> trailsWithoutDescMapper;

  this->Internal->TrailsWithDescPolyData->SetPoints(
    this->Internal->PointsWithDesc.GetPointer());

  this->Internal->TrailsWithDescPolyData->SetLines(
    this->Internal->TrailsWithDescCells.GetPointer());

  this->Internal->TrailsWithoutDescPolyData->SetPoints(
    this->Internal->PointsWithoutDesc.GetPointer());

  this->Internal->TrailsWithoutDescPolyData->SetLines(
    this->Internal->TrailsWithoutDescCells.GetPointer());

  trailsWithDescMapper->SetInputData(this->Internal->TrailsWithDescPolyData.GetPointer());
  trailsWithoutDescMapper->SetInputData(this->Internal->TrailsWithoutDescPolyData.GetPointer());

  this->ActivePointsWithDescActor = vtkSmartPointer<vtkActor>::New();
  this->ActivePointsWithDescActor->SetMapper(pointsWithDescMapper.GetPointer());

  this->ActivePointsWithoutDescActor = vtkSmartPointer<vtkActor>::New();
  this->ActivePointsWithoutDescActor->SetMapper(pointsWithoutDescMapper.GetPointer());

  this->TrailsWithDescActor = vtkSmartPointer<vtkActor>::New();
  this->TrailsWithDescActor->SetMapper(trailsWithDescMapper.GetPointer());

  this->TrailsWithoutDescActor = vtkSmartPointer<vtkActor>::New();
  this->TrailsWithoutDescActor->SetMapper(trailsWithoutDescMapper.GetPointer());
  this->TrailStyle = Historic;
}

//-----------------------------------------------------------------------------
vtkMaptkFeatureTrackRepresentation::~vtkMaptkFeatureTrackRepresentation()
{
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::AddTrackWithDescPoint(
  kwiver::vital::track_id_t trackId, kwiver::vital::frame_id_t frameId,
  double x, double y)
{
  auto const id = this->Internal->PointsWithDesc->InsertNextPoint(x, y, 0.0);
  this->Internal->TracksWithDesc[trackId][frameId] = id;
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::AddTrackWithoutDescPoint(
  kwiver::vital::track_id_t trackId, kwiver::vital::frame_id_t frameId,
  double x, double y)
{
  auto const id = this->Internal->PointsWithoutDesc->InsertNextPoint(x, y, 0.0);
  this->Internal->TracksWithoutDesc[trackId][frameId] = id;
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::ClearTrackData()
{
  this->Internal->PointsWithDesc->Reset();
  this->Internal->PointsWithoutDesc->Reset();
  this->Internal->TracksWithDesc.clear();
  this->Internal->TracksWithoutDesc.clear();
  this->Internal->PointsWithDescCells->Reset();
  this->Internal->PointsWithoutDescCells->Reset();
  this->Internal->TrailsWithDescCells->Reset();
  this->Internal->TrailsWithoutDescCells->Reset();
}

//-----------------------------------------------------------------------------
void vtkMaptkFeatureTrackRepresentation::SetActiveFrame(
  kwiver::vital::frame_id_t frame)
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
     << this->Internal->PointsWithDesc->GetNumberOfPoints() << endl;
  os << indent << "ActiveFrame: "
     << this->ActiveFrame << endl;
  os << indent << "TrailLength: "
     << this->TrailLength << endl;
}
