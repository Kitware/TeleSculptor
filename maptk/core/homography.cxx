/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core homography related classes implementation
 */

#include "homography.h"

namespace maptk
{


f2f_homography
::f2f_homography( const homography& h,
                  const frame_id_t from_id,
                  const frame_id_t to_id )
: homography(h),
  from_id_(from_id),
  to_id_(to_id)
{
}


f2f_homography
::f2f_homography( const f2f_homography& h )
: homography(h),
  from_id_(h.from_id()),
  to_id_(h.to_id())
{
}


f2f_homography
::~f2f_homography()
{
}


f2f_homography
f2f_homography
::inverse() const
{
  return f2f_homography( maptk::inverse( *this ), to_id_, from_id_ );
}


frame_id_t
f2f_homography
::from_id() const
{
  return from_id_;
}


frame_id_t
f2f_homography
::to_id() const
{
  return to_id_;
}


homography_collection
::homography_collection( f2f_homography_sptr cur_to_last,
                         f2f_homography_sptr cur_to_ref,
                         f2f_homography_sptr ref_to_wld )
: current_to_last_( cur_to_last ),
  current_to_reference_( cur_to_ref ),
  reference_to_world_( ref_to_wld )
{
}


homography_collection
::~homography_collection()
{
}


f2f_homography_sptr
homography_collection
::current_to_last() const
{
  return current_to_last_;
}


f2f_homography_sptr
homography_collection
::current_to_reference() const
{
  return current_to_reference_;
}


f2f_homography_sptr
homography_collection
::reference_to_world() const
{
  return reference_to_world_;
}


bool
homography_collection
::has_current_to_last() const
{
  return current_to_last_;
}

bool
homography_collection
::has_current_to_reference() const
{
  return current_to_reference_;
}

bool
homography_collection
::has_reference_to_world() const
{
  return reference_to_world_;
}


void
homography_collection
::set_current_to_last( f2f_homography_sptr h )
{
  current_to_last_ = h;
}

void
homography_collection
::set_current_to_reference( f2f_homography_sptr h )
{
  current_to_reference_ = h;
}

void
homography_collection
::set_reference_to_world( f2f_homography_sptr h )
{
  reference_to_world_ = h;
}


} // end namespace maptk
