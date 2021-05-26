// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKPOINTPLACER_H_
#define TELESCULPTOR_VTKMAPTKPOINTPLACER_H_

// VTK includes
#include <vtkPointPlacer.h>

// Forward declarations

class vtkMaptkPointPlacer : public vtkPointPlacer
{
public:
  vtkTypeMacro(vtkMaptkPointPlacer, vtkPointPlacer);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkPointPlacer* New();

  /**
   * Given a renderer and a display position in pixel coordinates,
   * compute the world position and orientation where this point
   * will be placed. This method is typically used by the
   * representation to place the point initially. A return value of 1
   * indicates that constraints of the placer are met.
   */
  using vtkPointPlacer::ComputeWorldPosition;
  virtual int ComputeWorldPosition(vtkRenderer* ren,
                                   double displayPos[2],
                                   double worldPos[3],
                                   double worldOrient[9]) override;

protected:
  vtkMaptkPointPlacer() = default;
  ~vtkMaptkPointPlacer() = default;

private:
  vtkMaptkPointPlacer(const vtkMaptkPointPlacer&) = delete;
  void operator=(const vtkMaptkPointPlacer) = delete;
};

#endif // vtkMaptkPointPlacer_h
