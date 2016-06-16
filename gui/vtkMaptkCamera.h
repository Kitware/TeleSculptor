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

#ifndef MAPTK_VTKMAPTKCAMERA_H_
#define MAPTK_VTKMAPTKCAMERA_H_

#include <vital/types/camera.h>

#include <vtkCamera.h>

class vtkMaptkCamera : public vtkCamera
{
public:
  vtkTypeMacro(vtkMaptkCamera, vtkCamera);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkMaptkCamera* New();

  // Description:
  // Set/Get the internal maptk camera
  kwiver::vital::camera_sptr GetCamera() const;
  void SetCamera(kwiver::vital::camera_sptr const& camera);

  // Description:
  // Project 3D point to 2D using the internal maptk camera
  bool ProjectPoint(kwiver::vital::vector_3d const& point,
                    double (&projPoint)[2]);

  // Description:
  // Update self (the VTK camera) based on the maptk camera and
  // ImageDimensions, if set
  bool Update();

  // Description:
  // Set/Get the dimensions (w x h) of the image which is used, with camera
  // instrinsics, to compute aspect ratio; if unavailable, an estimate is
  // extracted from the camera intrinsics (principal point).
  vtkGetVector2Macro(ImageDimensions, int);
  vtkSetVector2Macro(ImageDimensions, int);

  // Description:
  // Convenience method which calls the superclass method of same name using
  // the member AspectRatio.
  void GetFrustumPlanes(double planes[24]);

  // Description:
  // Compute the transformation matrix that projects the camera image space
  // onto the specified plane in world space
  void GetTransform(vtkMatrix4x4*, double const plane[4]);

  // Description:
  // Set/Get the aspect ratio (w / h) used when getting the frustum planes
  vtkGetMacro(AspectRatio, double);
  vtkSetMacro(AspectRatio, double);

  void DeepCopy(vtkMaptkCamera* source);

protected:
  vtkMaptkCamera();
  ~vtkMaptkCamera();

  using vtkCamera::GetFrustumPlanes; // Hide overloaded virtual

private:
  vtkMaptkCamera(const vtkMaptkCamera&); // Not implemented.
  void operator=(const vtkMaptkCamera&); // Not implemented.

  int ImageDimensions[2];
  double AspectRatio;

  kwiver::vital::camera_sptr MaptkCamera;
};

#endif
