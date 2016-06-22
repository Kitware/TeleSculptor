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

#include "MatchMatrixAlgorithms.h"

#include <cmath>

using std::log;
using std::pow;

//BEGIN value algorithms

//-----------------------------------------------------------------------------
double AbsoluteValueAlgorithm::operator()(
  Matrix const& /*unused*/, MatrixIterator const& iter) const
{
  return iter.value();
}

//-----------------------------------------------------------------------------
double AbsoluteValueAlgorithm::max(double maxRawValue) const
{
  return maxRawValue;
}

//-----------------------------------------------------------------------------
double AbstractRelativeValueAlgorithm::max(double /*maxRawValue*/) const
{
  return 1.0;
}

//-----------------------------------------------------------------------------
double RelativeXValueAlgorithm::operator()(
  Matrix const& matrix, MatrixIterator const& iter) const
{
  auto const k = iter.row();
  auto const d = static_cast<double>(matrix.coeff(k, k));
  auto const n = static_cast<double>(iter.value());
  return n / d;
}

//-----------------------------------------------------------------------------
double RelativeYValueAlgorithm::operator()(
  Matrix const& matrix, MatrixIterator const& iter) const
{
  auto const k = iter.col();
  auto const d = static_cast<double>(matrix.coeff(k, k));
  auto const n = static_cast<double>(iter.value());
  return n / d;
}

//-----------------------------------------------------------------------------
double RelativeXYValueAlgorithm::operator()(
  Matrix const& matrix, MatrixIterator const& iter) const
{
  auto const i = iter.row();
  auto const j = iter.col();
  auto const ni = static_cast<double>(matrix.coeff(i, i));
  auto const nj = static_cast<double>(matrix.coeff(j, j));
  auto const nc = static_cast<double>(iter.value());
  return nc / (ni + nj - nc);
}

//END value algorithms

///////////////////////////////////////////////////////////////////////////////

//BEGIN scale algorithms

//-----------------------------------------------------------------------------
LinearScaleAlgorithm::LinearScaleAlgorithm(double maxRawValue)
  : scale(1.0 / maxRawValue)
{
}

//-----------------------------------------------------------------------------
double LinearScaleAlgorithm::operator()(double rawValue) const
{
  return rawValue * this->scale;
}

//-----------------------------------------------------------------------------
LogarithmicScaleAlgorithm::LogarithmicScaleAlgorithm(
  double maxRawValue, double rangeScale)
  : preScale(rangeScale),
    postScale(1.0 / log((maxRawValue * rangeScale) + 1.0))
{
}

//-----------------------------------------------------------------------------
double LogarithmicScaleAlgorithm::operator()(double rawValue) const
{
  return log((rawValue * this->preScale) + 1.0) * this->postScale;
}

//-----------------------------------------------------------------------------
ExponentialScaleAlgorithm::ExponentialScaleAlgorithm(
  double maxRawValue, double exponent)
  : scale(1.0 / maxRawValue), exponent(exponent)
{
}

//-----------------------------------------------------------------------------
double ExponentialScaleAlgorithm::operator()(double rawValue) const
{
  return pow(rawValue * this->scale, this->exponent);
}

//END scale algorithms
