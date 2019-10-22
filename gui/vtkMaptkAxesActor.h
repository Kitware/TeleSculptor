/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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

#ifndef TELESCULPTOR_VTKMAPTKAXESACTOR_H_
#define TELESCULPTOR_VTKMAPTKAXESACTOR_H_

/**
 * @class vtkMaptkAxesActor
 * @brief Custom axes actor that is used when using constrained motion for
 * handle widgets in Telesculptor.
 */

// VTK includes
#include <vtkProp3D.h>

// Forward declarations
class vtkCylinderSource;
class vtkDiskSource;
class vtkLineSource;
class vtkSphereSource;
class vtkActor;

class vtkMaptkAxesActor : public vtkProp3D
{
public:
  vtkTypeMacro(vtkMaptkAxesActor, vtkProp3D);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkAxesActor* New();

  //@{
  /**
   * Support the standard render methods.
   */
  int RenderOpaqueGeometry(vtkViewport* viewport) override;
  int RenderTranslucentPolygonalGeometry(vtkViewport* viewport) override;
  int RenderOverlay(vtkViewport* viewport) override;
  //@}

  /**
   * Does this prop have some translucent polygonal geometry?
   */
  vtkTypeBool HasTranslucentPolygonalGeometry() override;

  /**
   * Shallow copy of an axes actor. Overloads the virtual vtkProp method.
   */
  void ShallowCopy(vtkProp* prop) override;

  /**
   * Release any graphics resources that are being consumed by this actor.
   * The parameter window could be used to determine which graphic
   * resources to release.
   */
  void ReleaseGraphicsResources(vtkWindow*) override;

  //@{
  /**
   * Get the bounds for this Actor as (Xmin,Xmax,Ymin,Ymax,Zmin,Zmax). (The
   * method GetBounds(double bounds[6]) is available from the superclass.)
   */
  void GetBounds(double bounds[6]);
  double* GetBounds() VTK_SIZEHINT(6) override;
  //@}

  //@{
  /**
   * Set/Get the Z axis visibility (Default: true)
   */
  vtkSetMacro(ZAxisVisibility, vtkTypeBool);
  vtkGetMacro(ZAxisVisibility, vtkTypeBool);
  vtkBooleanMacro(ZAxisVisibility, vtkTypeBool);
  //@}

  //@{
  /**
   * Set/Get the XY plane visibility (Default: true)
   */
  vtkSetMacro(XYPlaneVisibility, vtkTypeBool);
  vtkGetMacro(XYPlaneVisibility, vtkTypeBool);
  vtkBooleanMacro(XYPlaneVisibility, vtkTypeBool);
  //@}

  //@{
  /**
   * Set/Get the total length of the axes (Default: 1.5)
   */
  vtkSetMacro(AxesLength, int);
  vtkGetMacro(AxesLength, int);
  //@}

protected:
  vtkMaptkAxesActor();
  ~vtkMaptkAxesActor();

  // Member variables
  vtkCylinderSource* CylinderSource;
  vtkDiskSource* DiskSource;
  vtkLineSource* LineSource;
  vtkSphereSource* SphereSource;
  vtkActor* ZAxisShaft;
  vtkActor* ZAxisTip;
  vtkActor* XYPlaneDisk;
  vtkActor* XAxisShaft;
  vtkActor* XAxisTip;
  vtkActor* YAxisShaft;
  vtkActor* YAxisTip;

  vtkTypeBool ZAxisVisibility = true;
  vtkTypeBool XYPlaneVisibility = true;
  double AxesLength = 1.5;

  // Update the internal props
  void UpdateProps();

private:
  vtkMaptkAxesActor(const vtkMaptkAxesActor&) = delete;
  void operator=(const vtkMaptkAxesActor) = delete;
};

#endif
