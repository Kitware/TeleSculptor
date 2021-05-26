// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKDISTANCEWIDGET_H_
#define TELESCULPTOR_VTKMAPTKDISTANCEWIDGET_H_

// VTK includes
#include <vtkDistanceWidget.h>

// Forward declarations
class vtkMaptkDistanceWidgetCallback;
class vtkRenderWindowInteractor;

/**
 * @class vtkMaptkDistanceWidget
 * @brief Custom distance widget to add ruler support to Telesculptor
 *
 * The distance widget is used to place a ruler in the VTK view for measuring
 * distance between two points in the scene.  The vtkMaptkDistanceWidget differs
 * from the vtkDistanceWidget in two ways:
 *
 *  - It adds custom interaction constrains on the endpoints.  The user can
 *  enable/disable constrained motion as well as modify the constraint axis
 *  based for the ruler endpoints by holding shortcut keys:
 *
 *    - 'z':  Constrain motion of ruler endpoints along the Z direction
 *    - 'x' or 'y': Constrain motion of ruler endpoints along the XY plane
 *    - 'Esc': Remove any constraints on the endpoints
 *
 *  - It invokes the DistanceInteraction event when the user interacts with the
 *  endpoint.
 */
class vtkMaptkDistanceWidget : public vtkDistanceWidget
{
public:
  vtkTypeMacro(vtkMaptkDistanceWidget, vtkDistanceWidget);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkDistanceWidget* New();

  /**
   * Set the interactor
   */
  void SetInteractor(vtkRenderWindowInteractor* iren) override;

  int DistanceInteractionEvent;

protected:
  vtkMaptkDistanceWidget();
  ~vtkMaptkDistanceWidget();

  // Methods invoked when the handles at the
  // end points of the widget are manipulated
  void DistanceInteraction(int handleNum);

  friend class vtkMaptkDistanceWidgetCallback;
  vtkMaptkDistanceWidgetCallback* MaptkDistanceWidgetCallback1;
  vtkMaptkDistanceWidgetCallback* MaptkDistanceWidgetCallback2;

  // Member variables
  int ConstraintMode = -1;
  void SetConstraintMode(int mode);

  // Update the constraint mode on representation
  void UpdateRepresentationConstraint(int id);

private:
  vtkMaptkDistanceWidget(const vtkMaptkDistanceWidget&) = delete;
  void operator=(const vtkMaptkDistanceWidget) = delete;
};

#endif
