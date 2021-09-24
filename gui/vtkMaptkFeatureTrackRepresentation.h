// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKFEATURETRACKREPRESENTATION_H_
#define TELESCULPTOR_VTKMAPTKFEATURETRACKREPRESENTATION_H_

#include <vital/vital_types.h>

#include <vtkCamera.h>
#include <vtkCollection.h>
#include <vtkSmartPointer.h>

#include <memory>

class vtkActor;

class vtkMaptkFeatureTrackRepresentation : public vtkObject
{
public:
  enum TrailStyleEnum
  {
    Historic,
    Symmetric,
  };

  vtkTypeMacro(vtkMaptkFeatureTrackRepresentation, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkFeatureTrackRepresentation* New();

  void AddTrackWithDescPoint(kwiver::vital::track_id_t trackId,
                             kwiver::vital::frame_id_t frameId,
                             double x, double y);

  void AddTrackWithoutDescPoint(kwiver::vital::track_id_t trackId,
                                kwiver::vital::frame_id_t frameId,
                                double x, double y);

  // Description:
  // Remove all track data
  void ClearTrackData();

  // Description:
  // Get/Set the active frame
  void SetActiveFrame(kwiver::vital::frame_id_t);
  vtkGetMacro(ActiveFrame, kwiver::vital::frame_id_t);

  // Description:
  // Get/Set the maximum number of adjacent feature points to display as
  // "trails". (default == 2)
  void SetTrailLength(unsigned);
  vtkGetMacro(TrailLength, unsigned);

  // Description:
  // Get/Set the maximum number of adjacent feature points to display as
  // "trails". (default == 2)
  void SetTrailStyle(TrailStyleEnum);
  vtkGetMacro(TrailStyle, TrailStyleEnum);

  // Description:
  // Update the inputs for each of the actors, effectively updating the actor
  // at the next render.
  void Update();

  // Description:
  // Get the actor representing the active points with descriptors of the feature tracks
  vtkActor* GetActivePointsWithDescActor() { return this->ActivePointsWithDescActor; }

  // Description:
  // Get the actor representing the active points of the feature tracks
  vtkActor* GetActivePointsWithoutDescActor() { return this->ActivePointsWithoutDescActor; }

  // Description:
  // Get the actor representing the "trails without descriptors" in the feature tracks
  vtkActor* GetTrailsWithoutDescActor() { return this->TrailsWithoutDescActor; }

  // Description:
  // Get the actor representing the "trails with descriptors" in the feature tracks
  vtkActor* GetTrailsWithDescActor() { return this->TrailsWithDescActor; }

protected:
  vtkMaptkFeatureTrackRepresentation();
  ~vtkMaptkFeatureTrackRepresentation() override;

private:
  vtkMaptkFeatureTrackRepresentation(vtkMaptkFeatureTrackRepresentation const&) = delete;
  void operator=(vtkMaptkFeatureTrackRepresentation const&) = delete;

  kwiver::vital::frame_id_t ActiveFrame;
  unsigned TrailLength;
  TrailStyleEnum TrailStyle;

  vtkSmartPointer<vtkActor> ActivePointsWithDescActor;
  vtkSmartPointer<vtkActor> ActivePointsWithoutDescActor;

  vtkSmartPointer<vtkActor> TrailsWithDescActor;
  vtkSmartPointer<vtkActor> TrailsWithoutDescActor;

  class vtkInternal;
  std::unique_ptr<vtkInternal> const Internal;
};

#endif
