// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_MATCHMATRIXALGORITHMS_H_
#define TELESCULPTOR_MATCHMATRIXALGORITHMS_H_

#include <qtGlobal.h>
#include <vital/vital_config.h>

#include <Eigen/SparseCore>

//BEGIN value algorithms

//-----------------------------------------------------------------------------
class AbstractValueAlgorithm
{
public:
  typedef Eigen::SparseMatrix<unsigned int> Matrix;
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
