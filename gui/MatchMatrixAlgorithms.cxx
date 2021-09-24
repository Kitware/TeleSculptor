// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
