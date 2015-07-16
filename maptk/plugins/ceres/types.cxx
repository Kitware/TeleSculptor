/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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

/**
 * \file
 * \brief Implementation of enum to/from string conversions
 */


#include <maptk/plugins/ceres/types.h>
#include <ceres/loss_function.h>


namespace maptk
{

namespace ceres
{

#define CASESTR(x) case x: return #x
#define STRENUM(x) if (value == #x) { *type = x; return true;}

/// Convert a string to uppercase
static void UpperCase(std::string* input)
{
  std::transform(input->begin(), input->end(), input->begin(), ::toupper);
}


/// Provide a string representation for a LossFunctionType value
const char*
LossFunctionTypeToString(LossFunctionType type)
{
  switch (type)
  {
    CASESTR(TRIVIAL_LOSS);
    CASESTR(HUBER_LOSS);
    CASESTR(SOFT_L_ONE_LOSS);
    CASESTR(CAUCHY_LOSS);
    CASESTR(ARCTAN_LOSS);
    CASESTR(TUKEY_LOSS);
    default:
      return "UNKNOWN";
  }
}


/// Parse a LossFunctionType value from a string or return false
bool
StringToLossFunctionType(std::string value, LossFunctionType* type)
{
  UpperCase(&value);
  STRENUM(TRIVIAL_LOSS);
  STRENUM(HUBER_LOSS);
  STRENUM(SOFT_L_ONE_LOSS);
  STRENUM(CAUCHY_LOSS);
  STRENUM(ARCTAN_LOSS);
  STRENUM(TUKEY_LOSS);
  return false;
}


/// Provide a string representation for a LensDisortionType value
const char*
LensDistortionTypeToString(LensDistortionType type)
{
  switch (type)
  {
    CASESTR(NO_DISTORTION);
    CASESTR(POLYNOMIAL_RADIAL_DISTORTION);
    CASESTR(POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION);
    CASESTR(RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
    default:
      return "UNKNOWN";
  }
}


/// Parse a LensDistortionType value from a string or return false
bool
StringToLensDistortionType(std::string value, LensDistortionType* type)
{
  UpperCase(&value);
  STRENUM(NO_DISTORTION);
  STRENUM(POLYNOMIAL_RADIAL_DISTORTION);
  STRENUM(POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION);
  STRENUM(RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
  return false;
}

#undef CASESTR
#undef STRENUM


/// Construct a LossFunction object from the specified enum type
::ceres::LossFunction*
LossFunctionFactory(LossFunctionType type, double s)
{
  switch(type)
  {
    case TRIVIAL_LOSS:
      return NULL;
    case HUBER_LOSS:
      return new ::ceres::HuberLoss(s);
    case SOFT_L_ONE_LOSS:
      return new ::ceres::SoftLOneLoss(s);
    case CAUCHY_LOSS:
      return new ::ceres::CauchyLoss(s);
    case ARCTAN_LOSS:
      return new ::ceres::ArctanLoss(s);
    case TUKEY_LOSS:
      return new ::ceres::TukeyLoss(s);
    default:
      return NULL;
  }
}


} // end namespace ceres

} // end namespace maptk
