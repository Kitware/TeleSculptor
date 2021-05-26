// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKDISTANCEREPRESENTATION2D_H_
#define TELESCULPTOR_VTKMAPTKDISTANCEREPRESENTATION2D_H_

// VTK includes
#include <vtkDistanceRepresentation2D.h>

class vtkMaptkDistanceRepresentation2D : public vtkDistanceRepresentation2D
{
public:
  vtkTypeMacro(vtkMaptkDistanceRepresentation2D, vtkDistanceRepresentation2D);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkDistanceRepresentation2D* New();

  /**
   * Set the distance measurement
   */
  vtkSetMacro(Distance, double);

  /**
   * Override to use the pre-set distance measurement
   */
  void BuildRepresentation() override;

  /**
   * Set/Get whether to compute the distance label
   */
  vtkSetMacro(ComputeDistance, vtkTypeBool);
  vtkGetMacro(ComputeDistance, vtkTypeBool);
  vtkBooleanMacro(ComputeDistance, vtkTypeBool);

protected:
  vtkMaptkDistanceRepresentation2D();
  ~vtkMaptkDistanceRepresentation2D() = default;

  vtkTypeBool ComputeDistance = true;

private:
  vtkMaptkDistanceRepresentation2D(
    const vtkMaptkDistanceRepresentation2D&) = delete;
  void operator=(const vtkMaptkDistanceRepresentation2D) = delete;
};

#endif
