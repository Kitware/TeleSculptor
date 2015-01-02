/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief OCV descriptor_set implementation
 */

#include "descriptor_set.h"


/// This macro applies another macro to all of the types listed below.
#define APPLY_TO_TYPES(MACRO) \
    MACRO(byte); \
    MACRO(float); \
    MACRO(double)

namespace
{

/// Templated helper function to convert matrix row into a descriptor
template <typename T>
maptk::descriptor_sptr
ocv_to_maptk_descriptor(const cv::Mat& v)
{
  using namespace maptk;
  descriptor_array_of<T>* d = NULL;
  switch(v.cols)
  {
  case 64:
    d = new descriptor_fixed<T,64>();
    break;
  case 128:
    d = new descriptor_fixed<T,128>();
    break;
  case 256:
    d = new descriptor_fixed<T,256>();
    break;
  default:
    d = new descriptor_dynamic<T>(v.cols);
  }
  std::copy(v.begin<T>(), v.end<T>(), d->raw_data());
  return descriptor_sptr(d);
}


/// Templated helper function to convert descriptors into a cv::Mat
template <typename T>
cv::Mat
maptk_descriptors_to_ocv(const std::vector<maptk::descriptor_sptr>& desc)
{
  using namespace maptk;
  const unsigned int num = static_cast<unsigned int>(desc.size());
  const unsigned int dim = static_cast<unsigned int>(desc[0]->size());
  cv::Mat_<T> mat(num,dim);
  for( unsigned int i=0; i<num; ++i )
  {
    const descriptor_array_of<T>* d =
        dynamic_cast<const descriptor_array_of<T>*>(desc[i].get());
    if( !d || d->size() != dim )
    {
      std::cerr << "Error: mismatch type or size "
                << "when converting descriptors to OpenCV"
                << std::endl;
      return cv::Mat();
    }
    cv::Mat_<T> row = mat.row(i);
    std::copy(d->raw_data(), d->raw_data() + dim, row.begin());
  }
  return mat;
}

} // end anonymous namespace


namespace maptk
{

namespace ocv
{

/// Return a vector of descriptor shared pointers
std::vector<descriptor_sptr>
descriptor_set
::descriptors() const
{
  std::vector<descriptor_sptr> desc;
  const unsigned num_desc = data_.rows;
  /// \cond DoxygenSuppress
#define CONVERT_CASE(T) \
  case cv::DataType<T>::type: \
  for( unsigned i=0; i<num_desc; ++i ) \
  { \
    desc.push_back(ocv_to_maptk_descriptor<T>(data_.row(i))); \
  } \
  break

  switch(data_.type())
  {
  APPLY_TO_TYPES(CONVERT_CASE);
  default:
    std::cerr << "Error: No case to handle OpenCV descriptors of type "
              << data_.type() <<std::endl;
  }
#undef CONVERT_CASE
  /// \endcond
  return desc;
}


/// Convert any descriptor set to an OpenCV cv::Mat
cv::Mat
descriptors_to_ocv_matrix(const maptk::descriptor_set& desc_set)
{
  // if the descriptor set already contains a cv::Mat representation
  // then return the existing matrix
  if( const ocv::descriptor_set* d =
          dynamic_cast<const ocv::descriptor_set*>(&desc_set) )
  {
    return d->ocv_desc_matrix();
  }
  std::vector<descriptor_sptr> desc = desc_set.descriptors();
  if( desc.empty() )
  {
    return cv::Mat();
  }
  /// \cond DoxygenSuppress
#define CONVERT_CASE(T) \
  if( dynamic_cast<const descriptor_array_of<T>*>(desc[0].get()) ) \
  { \
    return maptk_descriptors_to_ocv<T>(desc); \
  }
  APPLY_TO_TYPES(CONVERT_CASE);
#undef CONVERT_CASE
  /// \endcond
  return cv::Mat();
}


} // end namespace ocv

} // end namespace maptk
