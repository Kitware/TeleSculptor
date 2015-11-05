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

#ifndef MAPTK_vtkMaptkCameraRepresentation_H_
#define MAPTK_vtkMaptkCameraRepresentation_H_

#include <vtkCamera.h>
#include <vtkCollection.h>
#include <vtkSmartPointer.h>

class vtkActor;
class vtkAppendPolyData;
class vtkPolyData;
class vtkTubeFilter;

class vtkMaptkCameraRepresentation : public vtkObject
{
public:
  vtkTypeMacro(vtkMaptkCameraRepresentation, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkMaptkCameraRepresentation *New();

  void AddCamera(vtkCamera* camera);
  void RemoveCamera(vtkCamera* camera);

  // Description:
  // Get/Set the camera to be displayed as the active camera
  void SetActiveCamera(vtkCamera* camera);
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
  // Get/Set the "skip" value for displaying cameras in the non-active camera
  // actor. A value of 0 means there is no skipping, 1 displays every other
  // one, 2 displays 1 of every 3, etc... (default == 5)
  vtkGetMacro(DisplaySkip, int);
  vtkSetMacro(DisplaySkip, int);

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
  ~vtkMaptkCameraRepresentation();

private:
  vtkMaptkCameraRepresentation(vtkMaptkCameraRepresentation const&) = delete;
  void operator=(vtkMaptkCameraRepresentation const&) = delete;

  double ActiveCameraRepLength;
  double NonActiveCameraRepLength;
  int DisplaySkip;

  vtkCamera* ActiveCamera;

  vtkActor* ActiveActor;
  vtkActor* NonActiveActor;
  vtkActor* PathActor;

  class vtkInternal;
  vtkInternal* const Internal;
};

#endif
