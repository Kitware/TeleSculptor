/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_MATCH_SET_H_
#define MAPTK_OCV_MATCH_SET_H_


#include <maptk/core/match_set.h>
#include <opencv2/features2d/features2d.hpp>

namespace maptk
{

namespace ocv
{


/// A concrete match set that wraps OpenCV cv::DMatch objects
class match_set
: public maptk::match_set
{
public:
  /// Default constructor
  match_set() {}

  /// Constructor from a vector of cv::DMatch
  explicit match_set(const std::vector<cv::DMatch>& matches)
  : data_(matches) {}

  /// Return the number of matches in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of matching indices
  virtual std::vector<match> matches() const;

  /// Return the underlying OpenCV match data structures
  const std::vector<cv::DMatch>& ocv_matches() const { return data_; }

private:
  // The vector of OpenCV match structures
  std::vector<cv::DMatch> data_;
};


/// Convert any match set to a vector of OpenCV cv::DMatch
std::vector<cv::DMatch>
matches_to_ocv_dmatch(const maptk::match_set& match_set);


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_MATCH_SET_H_
