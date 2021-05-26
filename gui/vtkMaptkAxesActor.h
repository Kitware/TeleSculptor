// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
  vtkSetMacro(AxesLength, double);
  vtkGetMacro(AxesLength, double);
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
