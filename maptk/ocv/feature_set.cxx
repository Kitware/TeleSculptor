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
    feature_f *f = new feature_f();
    f->set_loc(vector_2f(kp.pt.x, kp.pt.y));
    f->set_magnitude(kp.response);
    f->set_scale(kp.size);
    f->set_angle(kp.angle);
    features.push_back(feature_sptr(f));
  }
  return features;
}


/// Convert any feature set to a vector of OpenCV cv::KeyPoints
std::vector<cv::KeyPoint>
features_to_ocv_keypoints(const maptk::feature_set& feat_set)
{
  if( const ocv::feature_set* f =
          dynamic_cast<const ocv::feature_set*>(&feat_set) )
  {
    return f->ocv_keypoints();
  }
  std::vector<cv::KeyPoint> kpts;
  std::vector<feature_sptr> feat = feat_set.features();
  typedef std::vector<feature_sptr>::const_iterator feat_itr;
  for(feat_itr it = feat.begin(); it != feat.end(); ++it)
  {
    const feature_sptr f = *it;
    cv::KeyPoint kp;
    vector_2d pt = f->loc();
    kp.pt.x = pt.x();
    kp.pt.y = pt.y();
    kp.response = f->magnitude();
    kp.size = f->scale();
    kp.angle = f->angle();
    kpts.push_back(kp);
  }
  return kpts;
}


} // end namespace ocv

} // end namespace maptk
