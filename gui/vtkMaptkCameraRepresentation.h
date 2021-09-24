// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKCAMERAREPRESENTATION_H_
#define TELESCULPTOR_VTKMAPTKCAMERAREPRESENTATION_H_

#include <vital/vital_types.h>

#include <vtkCamera.h>
#include <vtkCollection.h>
#include <vtkSmartPointer.h>

#include <memory>

class vtkActor;

class vtkMaptkCameraRepresentation : public vtkObject
{
public:
  vtkTypeMacro(vtkMaptkCameraRepresentation, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkCameraRepresentation* New();

  void AddCamera(kwiver::vital::frame_id_t id, vtkCamera* camera);
  void RemoveCamera(kwiver::vital::frame_id_t id);

  // Mark the existing cameras as modified
  void CamerasModified();

  // Description:
  // Get/Set the camera to be displayed as the active camera
  void SetActiveCamera(kwiver::vital::frame_id_t id);
  vtkGetObjectMacro(ActiveCamera, vtkCamera);

  // Description:
  // Get/Set the distance to the far clipping plane of the active camera actor.
  // (default == 15)
  vtkGetMacro(ActiveCameraRepLength, double);
  vtkSetMacro(ActiveCameraRepLength, double);

  // Description:
  // Get/Set the distance to the far clipping plane of the non-active cameras
  // actor. (default == 4)
  vtkGetMacro(NonActiveCameraRepLength, double);
  vtkSetMacro(NonActiveCameraRepLength, double);

  // Description:
  // Get/Set the "density" value for displaying cameras in the non-active
  // camera actor. A value of 1 means to display all cameras, 2 displays every
  // other one, N displays 1 of every N, etc... (default == 1)
  vtkGetMacro(DisplayDensity, int);
  vtkSetMacro(DisplayDensity, int);

  // Description:
  // Update the inputs for each of the actors, effectively updating the actor
  // at the next render.
  void Update();

  // Description:
  // Get the actor representing the ActiveCamera (if any)
  vtkGetObjectMacro(ActiveActor, vtkActor);

  // Description:
  // Get the actor representing the non-active cameras
  vtkGetObjectMacro(NonActiveActor, vtkActor);

  // Description:
  // Get the actor representing the "path" of the cameras
  vtkGetObjectMacro(PathActor, vtkActor);

protected:
  vtkMaptkCameraRepresentation();
  ~vtkMaptkCameraRepresentation() override;

private:
  vtkMaptkCameraRepresentation(vtkMaptkCameraRepresentation const&) = delete;
  void operator=(vtkMaptkCameraRepresentation const&) = delete;

  double ActiveCameraRepLength;
  double NonActiveCameraRepLength;
  int DisplayDensity;

  vtkCamera* ActiveCamera;

  vtkActor* ActiveActor;
  vtkActor* NonActiveActor;
  vtkActor* PathActor;

  class vtkInternal;
  std::unique_ptr<vtkInternal> const Internal;
};

#endif
