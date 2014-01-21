/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/ocv/descriptor_set.h>

/// This macro applies another macro to all of the types listed below.
/// The listed type are all the types supported for conversion between
/// cv::Mat and maptk::descriptor
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
#define CONVERT_CASE(T) \
  if( dynamic_cast<const descriptor_array_of<T>*>(desc[0].get()) ) \
  { \
    return maptk_descriptors_to_ocv<T>(desc); \
  }
  APPLY_TO_TYPES(CONVERT_CASE);
#undef CONVERT_CASE
  return cv::Mat();
}


} // end namespace ocv

} // end namespace maptk
