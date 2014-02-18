/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_MATCH_SET_H_
#define MAPTK_MATCH_SET_H_


#include <vector>
#include <boost/shared_ptr.hpp>

namespace maptk
{

/// Standard match pairing
typedef std::pair<unsigned, unsigned> match;

/// A collection of matching indices between one set of objects and another.
class match_set
{
public:
  /// Destructor
  virtual ~match_set() {}

  /// Return the number of matches in the set
  virtual size_t size() const = 0;

  /// Return a vector of matching indices
  virtual std::vector<match> matches() const = 0;
};

/// Shared pointer of base match_set type
typedef boost::shared_ptr<match_set> match_set_sptr;


/// A concrete match set that simply wraps a vector of matches.
class simple_match_set
: public match_set
{
public:
  /// Default Constructor
  simple_match_set() {}

  /// Constructor from a vector of matches
  explicit simple_match_set(const std::vector<match>& matches)
  : data_(matches) {}

  /// Return the number of matches in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of match shared pointers
  virtual std::vector<match> matches() const { return data_; }

protected:

  /// The vector of matches
  std::vector<match> data_;
};


} // end namespace maptk


#endif // MAPTK_MATCH_SET_H_
