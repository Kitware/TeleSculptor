/*ckwg +29
 * Copyright 2016-2018 by Kitware, Inc.
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

#ifndef MAPTK_MATCHMATRIXALGORITHMS_H_
#define MAPTK_MATCHMATRIXALGORITHMS_H_

#include <qtGlobal.h>
#include <vital/vital_config.h>

#include <Eigen/SparseCore>

//BEGIN value algorithms

//-----------------------------------------------------------------------------
class AbstractValueAlgorithm
{
public:
  typedef Eigen::SparseMatrix<uint> Matrix;
  typedef Matrix::InnerIterator MatrixIterator;

  AbstractValueAlgorithm() = default;
  virtual ~AbstractValueAlgorithm() = default;

  AbstractValueAlgorithm(AbstractValueAlgorithm const&) = delete;
  AbstractValueAlgorithm& operator=(AbstractValueAlgorithm const&) = delete;

  virtual double operator()(Matrix const&, MatrixIterator const&) const = 0;
  virtual double max(double maxRawValue) const = 0;
};

//-----------------------------------------------------------------------------
class AbsoluteValueAlgorithm : public AbstractValueAlgorithm
{
  double operator()(Matrix const&, MatrixIterator const&) const override;
  double max(double maxRawValue) const override;
};

//-----------------------------------------------------------------------------
class AbstractRelativeValueAlgorithm : public AbstractValueAlgorithm
{
  double max(double maxRawValue) const override;
};

//-----------------------------------------------------------------------------
class RelativeXValueAlgorithm : public AbstractRelativeValueAlgorithm
{
  double operator()(Matrix const&, MatrixIterator const&) const override;
};

//-----------------------------------------------------------------------------
class RelativeYValueAlgorithm : public AbstractRelativeValueAlgorithm
{
  double operator()(Matrix const&, MatrixIterator const&) const override;
};

//-----------------------------------------------------------------------------
class RelativeXYValueAlgorithm : public AbstractRelativeValueAlgorithm
{
  double operator()(Matrix const&, MatrixIterator const&) const override;
};

//END value algorithms

///////////////////////////////////////////////////////////////////////////////

//BEGIN scale algorithms

//-----------------------------------------------------------------------------
class AbstractScaleAlgorithm
{
public:
  AbstractScaleAlgorithm() = default;
  virtual ~AbstractScaleAlgorithm() = default;

  AbstractScaleAlgorithm(AbstractScaleAlgorithm const&) = delete;
  AbstractScaleAlgorithm& operator=(AbstractScaleAlgorithm const&) = delete;

  virtual double operator()(double rawValue) const = 0;
};

//-----------------------------------------------------------------------------
class LinearScaleAlgorithm : public AbstractScaleAlgorithm
{
public:
  LinearScaleAlgorithm(double maxRawValue);

  double operator()(double rawValue) const override;

protected:
  double const scale;
};

//-----------------------------------------------------------------------------
class LogarithmicScaleAlgorithm : public AbstractScaleAlgorithm
{
public:
  LogarithmicScaleAlgorithm(double maxRawValue, double rangeScale = 1.0);

  double operator()(double rawValue) const override;

protected:
  double const preScale;
  double const postScale;
};

//-----------------------------------------------------------------------------
class ExponentialScaleAlgorithm : public AbstractScaleAlgorithm
{
public:
  ExponentialScaleAlgorithm(double maxRawValue, double exponent);

  double operator()(double rawValue) const override;

protected:
  double const scale;
  double const exponent;
};

//END scale algorithms

#endif
