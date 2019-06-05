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

class vtkMaptkDistanceWidget : public vtkDistanceWidget
{
public:
  vtkTypeMacro(vtkMaptkDistanceWidget, vtkDistanceWidget);
  void PrintSelf(ostream &os, vtkIndent indent) override;

  static vtkMaptkDistanceWidget* New();

  int DistanceInteractionEvent;
protected:
  vtkMaptkDistanceWidget();
  ~vtkMaptkDistanceWidget();

  // Methods invoked when the handles at the
  // end points of the widget are manipulated
  void DistanceInteraction(int handleNum);

  friend class vtkMaptkDistanceWidgetCallback;
  vtkMaptkDistanceWidgetCallback *MaptkDistanceWidgetCallback1;
  vtkMaptkDistanceWidgetCallback *MaptkDistanceWidgetCallback2;

private:
  vtkMaptkDistanceWidget(const vtkMaptkDistanceWidget&) = delete;
  void operator=(const vtkMaptkDistanceWidget) = delete;
};

#endif
