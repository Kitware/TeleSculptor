/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/ocv/descriptor_set.h>


namespace
{

template <typename T>
maptk::descriptor_sptr
ocv_to_maptk_descriptor(const cv::Mat& v)
{
  using namespace maptk;
  descriptor_array_of<T>* d = NULL;
  switch(v.rows)
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
    d = new descriptor_dynamic<T>(v.rows);
  }
  std::copy(v.begin<T>(), v.end<T>(), d->raw_data());
  return descriptor_sptr(d);
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
  const unsigned desc_dim = data_.cols;
#define CONVERT_CASE(T) \
  case cv::DataType<T>::type: \
  for( unsigned i=0; i<num_desc; ++i ) \
  { \
    desc.push_back(ocv_to_maptk_descriptor<T>(data_.row(i))); \
  } \
  break

  switch(data_.type())
  {
  CONVERT_CASE(byte);
  CONVERT_CASE(float);
  CONVERT_CASE(double);
  default:
    std::cerr << "Error: No case to handle OpenCV descriptors of type "
              << data_.type() <<std::endl;
  }
#undef CONVERT_CASE
  return desc;
}


} // end namespace ocv

} // end namespace maptk
