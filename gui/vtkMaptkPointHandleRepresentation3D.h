// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKPOINTHANDLEREPRESENTATION3D_H_
#define TELESCULPTOR_VTKMAPTKPOINTHANDLEREPRESENTATION3D_H_

// Telesculptor includes
#include "vtkMaptkAxesActor.h"

// VTK includes
#include <vtkPointHandleRepresentation3D.h>

// Forward declarations
class vtkRenderer;

class vtkMaptkPointHandleRepresentation3D
  : public vtkPointHandleRepresentation3D
{
public:
  vtkTypeMacro(vtkMaptkPointHandleRepresentation3D,
               vtkPointHandleRepresentation3D);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkPointHandleRepresentation3D* New();

  /**
   * Override widget interaction for custom constraints
   */
  void WidgetInteraction(double eventPos[2]) override;

  /**
   * Override build representation to show custom constraint axes
   */
  void BuildRepresentation() override;

  //@{
  /**
   * Set/Get whether to use a custom constraint.
   * In this mode, the constraint is exercised either along the Z axis or along
   * the XY plane. Defaults to false.
   */
  vtkSetMacro(CustomConstraint, int);
  vtkGetMacro(CustomConstraint, int);
  vtkBooleanMacro(CustomConstraint, int);
  //@}

  //@{
  /**
   * Override shallow and deep copy to add custom constraint.
   */
  void ShallowCopy(vtkProp* p) override;
  void DeepCopy(vtkProp* p) override;
  //@}

  //@{
  /**
   * Set/Get the axis to be constrained.
   */
  vtkSetMacro(ConstraintAxis, int);
  vtkGetMacro(ConstraintAxis, int);
  //@}

protected:
  vtkMaptkPointHandleRepresentation3D() = default;
  ~vtkMaptkPointHandleRepresentation3D() = default;

  // Override to ensure that the pick tolerance is always about the same as
  // handle size.
  int ComputeInteractionState(int X, int Y, int modify) override;

  // Constrained move focus
  void MoveFocusConstrained(double* p1, double* p2);

  // Constrained translate
  void TranslateConstrained(double* p1, double* p2, double* displaypos);

  /**
   * Update the axes
   */
  void UpdateAxes();

  // Member variables
  int CustomConstraint = 0;
  vtkNew<vtkMaptkAxesActor> AxesActor;

private:
  vtkMaptkPointHandleRepresentation3D(
    const vtkMaptkPointHandleRepresentation3D&) = delete;
  void operator=(const vtkMaptkPointHandleRepresentation3D) = delete;
};

#endif
