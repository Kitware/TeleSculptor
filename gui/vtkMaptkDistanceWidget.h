/*ckwg +29
 * Copyright 2019 by Kitware, Inc.
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
 *  based for the ruler endpoints based on shortcut keys.
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
