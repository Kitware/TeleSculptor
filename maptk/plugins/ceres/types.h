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
 * \brief Define additional enum types in a similar style as Ceres
 */

#ifndef MAPTK_PLUGINS_CERES_TYPES_H_
#define MAPTK_PLUGINS_CERES_TYPES_H_


#include <maptk/plugins/ceres/ceres_config.h>
#include <string>
#include <ceres/ceres.h>

namespace maptk
{

namespace ceres
{

/// The various robust loss function supported in the config
enum LossFunctionType
{
  TRIVIAL_LOSS,
  HUBER_LOSS,
  SOFT_L_ONE_LOSS,
  CAUCHY_LOSS,
  ARCTAN_LOSS,
  TUKEY_LOSS
};

/// The various models for lens distortion supported in the config
enum LensDistortionType
{
  NO_DISTORTION,
  POLYNOMIAL_RADIAL_DISTORTION,
  POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION,
  RATIONAL_RADIAL_TANGENTIAL_DISTORTION
};


/// Provide a string representation for a LossFunctionType value
MAPTK_CERES_EXPORT const char*
LossFunctionTypeToString(LossFunctionType type);

/// Parse a LossFunctionType value from a string or return false
MAPTK_CERES_EXPORT bool
StringToLossFunctionType(std::string value, LossFunctionType* type);

/// Construct a LossFunction object from the specified enum type
MAPTK_CERES_EXPORT ::ceres::LossFunction*
LossFunctionFactory(LossFunctionType type, double scale=1.0);


/// Provide a string representation for a LensDisortionType value
MAPTK_CERES_EXPORT const char*
LensDistortionTypeToString(LensDistortionType type);

/// Parse a LensDistortionType value from a string or return false
MAPTK_CERES_EXPORT bool
StringToLensDistortionType(std::string value, LensDistortionType* type);



} // end namespace ceres

} // end namespace maptk


#endif // MAPTK_PLUGINS_CERES_TYPES_H_
