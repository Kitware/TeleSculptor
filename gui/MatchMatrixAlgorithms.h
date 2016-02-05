/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

  virtual ~AbstractValueAlgorithm() VITAL_DEFAULT_DTOR;

  virtual double operator()(Matrix const&, MatrixIterator const&) const = 0;
  virtual double max(double maxRawValue) const = 0;
};

//-----------------------------------------------------------------------------
class AbsoluteValueAlgorithm : public AbstractValueAlgorithm
{
  virtual double operator()(Matrix const&,
                            MatrixIterator const&) const QTE_OVERRIDE;
  virtual double max(double maxRawValue) const QTE_OVERRIDE;
};

//-----------------------------------------------------------------------------
class AbstractRelativeValueAlgorithm : public AbstractValueAlgorithm
{
  virtual double max(double maxRawValue) const QTE_OVERRIDE;
};

//-----------------------------------------------------------------------------
class RelativeXValueAlgorithm : public AbstractRelativeValueAlgorithm
{
  virtual double operator()(Matrix const&,
                            MatrixIterator const&) const QTE_OVERRIDE;
};

//-----------------------------------------------------------------------------
class RelativeYValueAlgorithm : public AbstractRelativeValueAlgorithm
{
  virtual double operator()(Matrix const&,
                            MatrixIterator const&) const QTE_OVERRIDE;
};

//-----------------------------------------------------------------------------
class RelativeXYValueAlgorithm : public AbstractRelativeValueAlgorithm
{
  virtual double operator()(Matrix const&,
                            MatrixIterator const&) const QTE_OVERRIDE;
};

//END value algorithms

///////////////////////////////////////////////////////////////////////////////

//BEGIN scale algorithms

//-----------------------------------------------------------------------------
class AbstractScaleAlgorithm
{
public:
  virtual ~AbstractScaleAlgorithm() VITAL_DEFAULT_DTOR

  virtual double operator()(double rawValue) const = 0;
};

//-----------------------------------------------------------------------------
class LinearScaleAlgorithm : public AbstractScaleAlgorithm
{
public:
  LinearScaleAlgorithm(double maxRawValue);

  virtual double operator()(double rawValue) const QTE_OVERRIDE;

protected:
  double const scale;
};

//-----------------------------------------------------------------------------
class LogarithmicScaleAlgorithm : public AbstractScaleAlgorithm
{
public:
  LogarithmicScaleAlgorithm(double maxRawValue, double rangeScale = 1.0);

  virtual double operator()(double rawValue) const QTE_OVERRIDE;

protected:
  double const preScale;
  double const postScale;
};

//-----------------------------------------------------------------------------
class ExponentialScaleAlgorithm : public AbstractScaleAlgorithm
{
public:
  ExponentialScaleAlgorithm(double maxRawValue, double exponent);

  virtual double operator()(double rawValue) const QTE_OVERRIDE;

protected:
  double const scale;
  double const exponent;
};

//END scale algorithms

#endif
