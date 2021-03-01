/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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

#include "vtkMaptkScalarsToGradient.h"

#include <vtkObjectFactory.h>
#include <vtkTemplateAliasMacro.h>

#include <qtGradient.h>
#include <qtIndexRange.h>

#include <cmath>

vtkStandardNewMacro(vtkMaptkScalarsToGradient);

QTE_IMPLEMENT_D_FUNC(vtkMaptkScalarsToGradient)

#if defined(_MSC_VER) && _MSC_VER <= 1800
#define __func__ __FUNCTION__
#endif

namespace // anonymous
{

typedef double (*GetValue)(void const*, int);

//-----------------------------------------------------------------------------
template <typename T>
double getValue(void const* data, int offset)
{
  auto const realData = static_cast<T const*>(data);
  return static_cast<double>(realData[offset]);
}

//-----------------------------------------------------------------------------
GetValue typedGetValue(int type)
{
  switch (type)
  {
    vtkTemplateAliasMacro(return &getValue<VTK_TT>);
    default:
      return 0;
  }
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class vtkMaptkScalarsToGradientPrivate
{
public:
  double lower;
  double scale;

  qtGradient gradient;
};

//-----------------------------------------------------------------------------
vtkMaptkScalarsToGradient::vtkMaptkScalarsToGradient()
  : d_ptr(new vtkMaptkScalarsToGradientPrivate)
{
  QTE_D();
  d->lower = 0.0;
  d->scale = 1.0;

  this->vtkScalarsToColors::SetRange(0.0, 1.0);
}

//-----------------------------------------------------------------------------
vtkMaptkScalarsToGradient::~vtkMaptkScalarsToGradient()
{
  delete this->d_ptr;
}

//-----------------------------------------------------------------------------
void vtkMaptkScalarsToGradient::SetGradient(qtGradient const& gradient)
{
  QTE_D();

  d->gradient = gradient;

  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkScalarsToGradient::SetRange(double min, double max)
{
  QTE_D();

  d->lower = min;
  double range = max - min;
  if (range <= 0.0)
  {
    range = 1.0;
  }
  d->scale = 1.0 / range;

  this->vtkScalarsToColors::SetRange(min, max);
}

//-----------------------------------------------------------------------------
void vtkMaptkScalarsToGradient::GetColor(double v, double rgb[3])
{
  QTE_D();

  auto const k = (v - d->lower) * d->scale;
  auto const c = d->gradient.at(k);

  rgb[0] = c.redF();
  rgb[1] = c.greenF();
  rgb[2] = c.blueF();
}

//-----------------------------------------------------------------------------
void vtkMaptkScalarsToGradient::MapScalarsThroughTable2(
  void* input, unsigned char* output, int inputDataType,
  int numberOfValues, int inputIncrement, int outputFormat)
{
  QTE_D();

  auto const get = typedGetValue(inputDataType);
  if (!get)
  {
    vtkErrorMacro(<< __func__ << ": Unknown input data type");
    return;
  }

  switch (outputFormat)
  {
    case VTK_RGBA:
      foreach (auto const i, qtIndexRange(numberOfValues))
      {
        auto const value = (*get)(input, i);
        auto const k = std::isfinite(value) ?
          (value - d->lower) * d->scale :
          d->lower;
        auto const c = d->gradient.at(k);

        auto const out = output + (4 * i);
        out[0] = c.red();
        out[1] = c.green();
        out[2] = c.blue();
        out[3] = c.alpha();
        if (!std::isfinite(value))
        {
          out[3] = 0;
        }
      }
      break;

    case VTK_RGB:
      foreach (auto const i, qtIndexRange(numberOfValues))
      {
        auto const value = (*get)(input, i);
        auto const k = std::isfinite(value) ?
          (value - d->lower) * d->scale :
          d->lower;
        auto const c = d->gradient.at(k);

        auto const out = output + (3 * i);
        out[0] = c.red();
        out[1] = c.green();
        out[2] = c.blue();
      }
      break;

    default:
      this->vtkScalarsToColors::MapScalarsThroughTable2(
        input, output, inputDataType, numberOfValues,
        inputIncrement, outputFormat);
      break;
  }
}
