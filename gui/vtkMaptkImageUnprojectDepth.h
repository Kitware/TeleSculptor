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

#ifndef vtkMaptkImageUnprojectDepth_h
#define vtkMaptkImageUnprojectDepth_h

#include "vtkSimpleImageToImageFilter.h"

namespace kwiver {
namespace arrows {
namespace vtk {
class vtkKwiverCamera;
}}}

class vtkMaptkImageUnprojectDepth : public vtkSimpleImageToImageFilter
{
public:
  static vtkMaptkImageUnprojectDepth *New();
  vtkTypeMacro(vtkMaptkImageUnprojectDepth,vtkSimpleImageToImageFilter);

  // Description:
  // Get/Set the camera to be displayed as the active camera
  void SetCamera(kwiver::arrows::vtk::vtkKwiverCamera* camera);
  vtkGetObjectMacro(Camera, kwiver::arrows::vtk::vtkKwiverCamera);

  // Description:
  // Get/Set the depth array name.  Default is "Depths".
  vtkSetStringMacro(DepthArrayName);
  vtkGetStringMacro(DepthArrayName);

  // Description:
  // Get/Set the unprojected point array name.  Default is "PointsWithDesc".
  vtkSetStringMacro(UnprojectedPointArrayName);
  vtkGetStringMacro(UnprojectedPointArrayName);

protected:
  vtkMaptkImageUnprojectDepth();
  ~vtkMaptkImageUnprojectDepth() override;

  void SimpleExecute(vtkImageData* input, vtkImageData* output) override;
private:
  vtkMaptkImageUnprojectDepth(vtkMaptkImageUnprojectDepth const&) = delete;
  void operator=(vtkMaptkImageUnprojectDepth const&) = delete;

  kwiver::arrows::vtk::vtkKwiverCamera* Camera;

  char* DepthArrayName;
  char* UnprojectedPointArrayName;
};

#endif







// VTK-HeaderTest-Exclude: vtkMaptkImageUnprojectDepth.h
