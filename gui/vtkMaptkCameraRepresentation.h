/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
