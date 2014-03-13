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
::f2f_homography( const frame_id_t frame_id )
: from_id_( frame_id ),
  to_id_( frame_id )
{
  this->set_identity();
}


f2f_homography
::f2f_homography( const homography& h,
                  const frame_id_t from_id,
                  const frame_id_t to_id )
: homography( h ),
  from_id_( from_id ),
  to_id_( to_id )
{
}


f2f_homography
::f2f_homography( const f2f_homography& h )
: homography( h ),
  from_id_( h.from_id() ),
  to_id_( h.to_id() )
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


f2w_homography
::f2w_homography( const frame_id_t frame_id )
: frame_id_( frame_id )
{
  this->set_identity();
}


f2w_homography
::f2w_homography( const homography& h,
                  const frame_id_t frame_id )
: homography( h ),
  frame_id_( frame_id )
{
}


f2w_homography
::f2w_homography( const f2w_homography& h )
: homography( h ),
  frame_id_( h.frame_id() )
{
}


f2w_homography
::~f2w_homography()
{
}


f2w_homography
f2w_homography
::inverse() const
{
  return f2w_homography( maptk::inverse( *this ), frame_id_ );
}


frame_id_t
f2w_homography
::frame_id() const
{
  return frame_id_;
}


homography_collection
::homography_collection( f2f_homography_sptr cur_to_last,
                         f2f_homography_sptr cur_to_ref,
                         f2w_homography_sptr ref_to_wld,
                         f2w_homography_sptr cur_to_wld )
: current_to_last_( cur_to_last ),
  current_to_reference_( cur_to_ref ),
  reference_to_world_( ref_to_wld ),
  current_to_world_( cur_to_wld )
{
  if( !current_to_world_ && current_to_reference_ && reference_to_world_ )
  {
    homography h = (*reference_to_world_) * (*current_to_reference_);
    frame_id_t from = current_to_reference_->from_id();
    current_to_world_ = f2w_homography_sptr( new f2w_homography( h, from ) );
  }
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


f2w_homography_sptr
homography_collection
::reference_to_world() const
{
  return reference_to_world_;
}


f2w_homography_sptr
homography_collection
::current_to_world() const
{
  return current_to_world_;
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


bool
homography_collection
::has_current_to_world() const
{
  return current_to_world_;
}


frame_id_t
homography_collection
::frame_id() const
{
  if( current_to_reference_ )
  {
    return current_to_reference_->from_id();
  }
  else if( current_to_last_ )
  {
    return current_to_last_->from_id();
  }
  else if( current_to_world_ )
  {
    return current_to_world_->frame_id();
  }
  throw invalid_data( "Collection is empty." );
}


homography_point
::homography_point()
{
  data_[0] = 0;
  data_[1] = 0;
}


homography_point
::homography_point( const double i, const double j )
{
  data_[0] = i;
  data_[1] = j;
}


homography_point
::homography_point( const vector_2d& v )
 : vector_2d( v )
{
}


double
homography_point
::i() const
{
  return data_[0];
}


double
homography_point
::j() const
{
  return data_[1];
}


vector_2d
homography_point
::loc() const
{
  return (*this);
}


homography_point
operator*( const homography& h, const homography_point& p )
{
  matrix_<3,1,double> mat_pt;
  mat_pt(0,0) = p.x(), mat_pt(1,0) = p.y(), mat_pt(2,0) = 1;

  matrix_<3,1,double> out_pt = h * mat_pt;

  if( out_pt(2,0) == 0 )
  {
    throw point_maps_to_infinity();
  }

  return homography_point( out_pt(0,0) / out_pt(2,0), out_pt(1,0) / out_pt(2,0) );
}

} // end namespace maptk
