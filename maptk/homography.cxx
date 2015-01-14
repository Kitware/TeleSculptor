/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief core homography related classes implementation
 */

#include "homography.h"
#include "exceptions/math.h"
#include <Eigen/LU>

namespace maptk
{


f2f_homography
::f2f_homography( const frame_id_t frame_id )
: from_id_( frame_id ),
  to_id_( frame_id )
{
  this->setIdentity();
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
  return f2f_homography( homography::inverse(), to_id_, from_id_ );
}


f2f_homography&
f2f_homography
::normalize()
{
  if( (*this)(2,2) != 0 )
  {
    (*this) /= (*this)(2,2);
  }
  return *this;
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


f2f_homography
f2f_homography
::operator*( const f2f_homography& other )
{
  if( this->from_id() != other.to_id() )
  {
    throw invalid_matrix_operation( "Frame homography identifiers do not match" );
  }

  return f2f_homography( static_cast<homography>(*this) *
                         static_cast<homography>(other),
                         other.from_id(), to_id_ );
}


f2w_homography
::f2w_homography( const frame_id_t frame_id )
: frame_id_( frame_id )
{
  this->setIdentity();
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
  return static_cast<bool>(current_to_last_);
}


bool
homography_collection
::has_current_to_reference() const
{
  return static_cast<bool>(current_to_reference_);
}


bool
homography_collection
::has_reference_to_world() const
{
  return static_cast<bool>(reference_to_world_);
}


bool
homography_collection
::has_current_to_world() const
{
  return static_cast<bool>(current_to_world_);
}


vector_2d
homography_map( const homography& h, const vector_2d& p )
{
  vector_3d out_pt = h * vector_3d(p[0], p[1], 1.0);

  if( out_pt[2] == 0 )
  {
    throw point_maps_to_infinity();
  }

  return vector_2d( out_pt[0] / out_pt[2], out_pt[1] / out_pt[2] );
}


} // end namespace maptk
