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

#ifndef MAPTK_VTKMKCAMERA_H_
#define MAPTK_VTKMKCAMERA_H_

#include <vtkCamera.h>

#include <maptk/camera.h>

class vtkMkCamera : public vtkCamera
{
public:
  vtkTypeMacro(vtkMkCamera, vtkCamera);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkMkCamera *New();

  // Description:
  // Set the internal maptk camera
  void SetCamera(maptk::camera_d camera);

  // Description:
  // Project 3D point to 2S using the internal maptk camera
  bool ProjectPoint(maptk::vector_3d point, double projPoint[2]);

  // Description:
  // Update self (the vtk camera) based on the maptk camera and 
  // ImageDimensions, if set
  bool Update();

  // Description:
  // Set/Get the dimensions (w x h) of the image which is used, with camera
  // instrinsics, to compute aspect ratio; if unavailable, an estimate is
  // extracted from the camera intrinsics (principal point).
  vtkGetVector2Macro(ImageDimensions, int);
  vtkSetVector2Macro(ImageDimensions, int);

  // Description:
  // Convenience fn which calls the superclass fn of same name using the
  // member AspectRatio.
  void GetFrustumPlanes(double planes[24]);

  // Description:
  // Set/Get the aspect ratio (w / h) used when getting the frustum planes
  vtkGetMacro(AspectRatio, double);
  vtkSetMacro(AspectRatio, double);

protected:
  vtkMkCamera();
  ~vtkMkCamera();

private:
  vtkMkCamera(const vtkMkCamera&);  // Not implemented.
  void operator=(const vtkMkCamera&);  // Not implemented.

  int ImageDimensions[2];
  double AspectRatio;

  maptk::camera_d MaptkCamera;

  class vtkInternal;
  vtkInternal* Internal;
};

#endif
