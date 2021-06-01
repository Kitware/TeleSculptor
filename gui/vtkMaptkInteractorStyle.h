// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKINTERACTORSTYLE_H
#define TELESCULPTOR_VTKMAPTKINTERACTORSTYLE_H

// VTK includes
#include <vtkInteractorStyleTrackballCamera.h>

class vtkMaptkInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  vtkTypeMacro(vtkMaptkInteractorStyle, vtkInteractorStyleTrackballCamera);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkInteractorStyle* New();

  virtual void OnLeftButtonDown() override;

protected:
  vtkMaptkInteractorStyle();
  ~vtkMaptkInteractorStyle() = default;

  enum Timer
  {
    Timing = 0,
    TimedOut,
  };

  int TimerStatus = TimedOut;
  int TimerId = -1;

  void TimerCallback(vtkObject*, unsigned long, void*);
  void DestroyTimer();

private:
  vtkMaptkInteractorStyle(vtkMaptkInteractorStyle const&) = delete;
  void operator=(vtkMaptkInteractorStyle const&) = delete;
};

#endif
