// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
