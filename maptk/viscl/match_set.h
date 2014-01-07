/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_MATCH_SET_H_
#define MAPTK_VISCL_MATCH_SET_H_


#include <maptk/core/match_set.h>

namespace maptk
{

namespace viscl
{


/// A concrete match set that wraps VisCL matches
class match_set
: public maptk::match_set
{
public:
  /// Default constructor
  match_set() {}

  /// Constructor from VisCL matches
  //TODO define 'type'
  //explicit match_set(const type& viscl_matches)
  //: data_(viscl_matches) {}

  /// Return the number of matches in the set
  virtual size_t size() const { return 0; } // TODO return size

  /// Return a vector of matching indices
  virtual std::vector<match> matches() const;

  /// Return the underlying VisCL match data
  // TODO define 'type'
  //const type& viscl_matches() const { return data_; }

private:
  // The collection of VisCL match data
  //TODO define 'type'
  //type data_;
};


/// Convert any match set to VisCL match data
//TODO define 'type'
//type
//matches_to_viscl(const maptk::match_set& match_set);


} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_MATCH_SET_H_
