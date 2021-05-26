// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKPOINTPICKER_H_
#define TELESCULPTOR_VTKMAPTKPOINTPICKER_H_

// VTK includes
#include <vtkNew.h>
#include <vtkPointPicker.h>

// Forward declarations
class vtkRenderer;

class vtkMaptkPointPicker : public vtkPointPicker
{
public:
  vtkTypeMacro(vtkMaptkPointPicker, vtkPointPicker);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkPointPicker* New();

  /*
   * Pick a point in the scene with the selection point and focal point
   * provided. The two points are in world coordinates.
   *
   * Returns non-zero if something was successfully picked.
   */
  using vtkPicker::Pick3DPoint;
  virtual int Pick3DPoint(double p1World[3],
                          double p2World[3],
                          vtkRenderer* ren);

protected:
  vtkMaptkPointPicker() = default;
  ~vtkMaptkPointPicker() = default;

private:
  vtkMaptkPointPicker(const vtkMaptkPointPicker&) = delete;
  void operator=(const vtkMaptkPointPicker) = delete;
};

#endif
