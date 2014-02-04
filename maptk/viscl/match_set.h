/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_MATCH_SET_H_
#define MAPTK_VISCL_MATCH_SET_H_


#include <maptk/core/match_set.h>
#include <viscl/core/buffer.h>

namespace maptk
{

namespace vcl
{


/// A concrete match set that wraps VisCL matches
class match_set
: public maptk::match_set
{
public:
  /// Default constructor
  match_set() {}

  /// Constructor from VisCL matches
  explicit match_set(const viscl::buffer& viscl_matches)
   : data_(viscl_matches) {}

  /// Return the number of matches in the set
  virtual size_t size() const { return 0; } // TODO return size

  /// Return a vector of matching indices
  virtual std::vector<match> matches() const;

  /// Return the underlying VisCL match data
  const viscl::buffer& viscl_matches() const { return data_; }

private:
  // The collection of VisCL match data
  viscl::buffer data_;
};


/// Convert any match set to VisCL match data
viscl::buffer
matches_to_viscl(const maptk::match_set& match_set, size_t numkpts2);


} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_MATCH_SET_H_
