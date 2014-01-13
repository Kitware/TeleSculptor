/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_LANDMARK_MAP_H_
#define MAPTK_LANDMARK_MAP_H_

/**
 * \file
 * \brief Header file for a map from IDs to landmarks
 */

#include "landmark.h"
#include <map>
#include <boost/shared_ptr.hpp>

namespace maptk
{

/// An abstract mapping between track IDs and landmarks
class landmark_map
{
public:
  /// typedef for std::map from integer IDs to landmarks
  typedef std::map<unsigned long, landmark_sptr> map_landmark_t;

  /// Destructor
  virtual ~landmark_map() {}

  /// Return the number of landmarks in the map
  virtual size_t size() const = 0;

  /// Return a map from integer IDs to landmark shared pointers
  virtual map_landmark_t landmarks() const = 0;
};

/// typedef for a landmark shared pointer
typedef boost::shared_ptr<landmark_map> landmark_map_sptr;


/// A concrete landmark_map that simply wraps a std::map.
class simple_landmark_map
: public landmark_map
{
public:
  /// Default Constructor
  simple_landmark_map() {}

  /// Constructor from a std::map of landmarks
  explicit simple_landmark_map(const map_landmark_t& landmarks)
  : data_(landmarks) {}

  /// Return the number of landmarks in the map
  virtual size_t size() const { return data_.size(); }

  /// Return a map from integer IDs to landmark shared pointers
  virtual map_landmark_t landmarks() const { return data_; }

protected:

  /// The map from integer IDs to landmark shared pointers
  map_landmark_t data_;
};


} // end namespace maptk


#endif // MAPTK_LANDMARK_MAP_H_
