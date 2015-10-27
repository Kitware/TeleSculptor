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

#ifndef MAPTK_VTKMAPTKFEATURETRACKREPRESENTATION_H_
#define MAPTK_VTKMAPTKFEATURETRACKREPRESENTATION_H_

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
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkMaptkFeatureTrackRepresentation *New();

  void AddTrackPoint(unsigned trackId, unsigned frameId, double x, double y);

  // Description:
  // Get/Set the active frame
  void SetActiveFrame(unsigned);
  vtkGetMacro(ActiveFrame, unsigned);

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
  // Get the actor representing the active points of the feature tracks
  vtkGetObjectMacro(ActivePointsActor, vtkActor);

  // Description:
  // Get the actor representing the "trails" of the feature tracks
  vtkGetObjectMacro(TrailsActor, vtkActor);

protected:
  vtkMaptkFeatureTrackRepresentation();
  ~vtkMaptkFeatureTrackRepresentation();

private:
  vtkMaptkFeatureTrackRepresentation(vtkMaptkFeatureTrackRepresentation const&) = delete;
  void operator=(vtkMaptkFeatureTrackRepresentation const&) = delete;

  unsigned ActiveFrame;
  unsigned TrailLength;
  TrailStyleEnum TrailStyle;

  vtkSmartPointer<vtkActor> ActivePointsActor;
  vtkSmartPointer<vtkActor> TrailsActor;

  class vtkInternal;
  std::unique_ptr<vtkInternal> const Internal;
};

#endif
