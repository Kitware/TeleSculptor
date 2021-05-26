// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VTKMAPTKSCALARSTOGRADIENT_H_
#define TELESCULPTOR_VTKMAPTKSCALARSTOGRADIENT_H_

#include <vtkScalarsToColors.h>

#include <qtGlobal.h>

class qtGradient;

class vtkMaptkScalarsToGradientPrivate;

class vtkMaptkScalarsToGradient : public vtkScalarsToColors
{
public:
  vtkTypeMacro(vtkMaptkScalarsToGradient, vtkScalarsToColors);
  static vtkMaptkScalarsToGradient* New();

  void SetGradient(qtGradient const&);

  using Superclass::SetRange;
  void SetRange(double min, double max) override;

  void GetColor(double v, double rgb[3]) override;

protected:
  QTE_DECLARE_PRIVATE_PTR(vtkMaptkScalarsToGradient)

  vtkMaptkScalarsToGradient();
  ~vtkMaptkScalarsToGradient() override;

  void MapScalarsThroughTable2(
    void* input, unsigned char* output, int inputDataType,
    int numberOfValues, int inputIncrement, int outputFormat) override;

private:
  QTE_DECLARE_PRIVATE(vtkMaptkScalarsToGradient)
  QTE_DISABLE_COPY(vtkMaptkScalarsToGradient)
};

#endif
