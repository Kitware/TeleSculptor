/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


#include <maptk/ocv/feature_set.h>

namespace maptk
{

namespace ocv
{


/// Return a vector of feature shared pointers
std::vector<feature_sptr>
feature_set
::features() const
{
  typedef std::vector<cv::KeyPoint>::const_iterator cvKP_itr;
  std::vector<feature_sptr> features;
  for(cvKP_itr it = data_.begin(); it != data_.end(); ++it)
  {
    const cv::KeyPoint& kp = *it;
    feature_<float> *f = new feature_<float>();
    f->set_loc(vector_2f(kp.pt.x, kp.pt.y));
    f->set_magnitude(kp.response);
    f->set_scale(kp.size);
    f->set_angle(kp.angle);
    features.push_back(feature_sptr(f));
  }
  return features;
}



} // end namespace ocv

} // end namespace maptk
