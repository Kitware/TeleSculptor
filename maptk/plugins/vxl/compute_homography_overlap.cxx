/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Implementation of homography overlap helper functions
 */

#include "compute_homography_overlap.h"

#include <algorithm>
#include <vector>

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_convex.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vgl/vgl_area.h>
#include <vgl/vgl_intersection.h>

#include <vnl/vnl_double_3.h>


namespace maptk
{

namespace vxl
{

namespace
{


// Does a given point occur inside a polygon?
bool does_intersect( const std::vector< vgl_point_2d<double> >& poly, const vgl_point_2d<double>& pt )
{
  vgl_polygon<double> pol( &poly[0], static_cast<int>(poly.size()) );
  return pol.contains( pt );
}


// Compute intersection points
void intersecting_points( const unsigned ni, const unsigned nj,
                          const std::vector< vgl_point_2d<double> >& opoly,
                          std::vector< vgl_point_2d<double> >& output )
{
  vgl_box_2d<double> borders( 0, ni, 0, nj );
  vgl_line_segment_3d<double> lines1[4];
  vgl_line_segment_3d<double> lines2[4];

  vgl_point_3d<double> poly[4];

  for( unsigned i = 0; i < opoly.size(); i++ )
  {
    poly[i] = vgl_point_3d<double>( opoly[i].x(), opoly[i].y(), 0 );
  }

  lines1[0] = vgl_line_segment_3d<double>( poly[0], poly[1] );
  lines1[1] = vgl_line_segment_3d<double>( poly[1], poly[2] );
  lines1[2] = vgl_line_segment_3d<double>( poly[2], poly[3] );
  lines1[3] = vgl_line_segment_3d<double>( poly[3], poly[0] );

  lines2[0] = vgl_line_segment_3d<double>( vgl_point_3d<double>( 0, 0, 0 ), vgl_point_3d<double>( ni, 0, 0 ) );
  lines2[1] = vgl_line_segment_3d<double>( vgl_point_3d<double>( ni, 0, 0 ), vgl_point_3d<double>( ni, nj, 0 ) );
  lines2[2] = vgl_line_segment_3d<double>( vgl_point_3d<double>( ni, nj, 0 ), vgl_point_3d<double>( 0, nj, 0 ) );
  lines2[3] = vgl_line_segment_3d<double>( vgl_point_3d<double>( 0, nj, 0 ), vgl_point_3d<double>( 0, 0, 0 ) );

  for( unsigned i = 0; i < 4; i++ )
  {
    for( unsigned j = 0; j < 4; j++ )
    {
      vgl_point_3d<double> new_pt;

      if( vgl_intersection( lines1[i], lines2[j], new_pt ) )
      {
        output.push_back( vgl_point_2d<double>( new_pt.x(), new_pt.y() ) );
      }
    }
  }
}


// Return area of a set of points
double convex_area( const std::vector< vgl_point_2d<double> >& poly )
{
  return vgl_area( vgl_convex_hull( poly ) );
}


// Compare two points, used for sorting.
bool compare_poly_pts( const vgl_point_2d<double>& p1, const vgl_point_2d<double>& p2 )
{
  if( p1.x() < p2.x() )
  {
    return true;
  }
  else if( p1.x() > p2.x() )
  {
    return false;
  }

  return ( p1.y() < p2.y() );
}


// Remove duplicates from the array while sorting it
void
sort_and_remove_duplicates( std::vector< vgl_point_2d<double> >& poly )
{
  if( poly.size() == 0 )
  {
    return;
  }

  std::sort( poly.begin(), poly.end(), compare_poly_pts );

  std::vector< vgl_point_2d<double> > filtered;
  filtered.push_back( poly[0] );

  for( unsigned i = 1; i < poly.size(); i++ )
  {
    if( poly[i] != poly[i-1] )
    {
      filtered.push_back( poly[i] );
    }
  }

  if( filtered.size() != poly.size() )
  {
    poly = filtered;
  }
}


} // end namespace anonymous


// Calculate percent overlap.
double
overlap( const vnl_double_3x3& h, const unsigned ni, const unsigned nj )
{
  // Early exit cases
  if( !ni || !nj )
  {
    return 0.0;
  }
  else if( h.is_identity() )
  {
    return 1.0;
  }

  // 1. Warp corner points to reference system [an image from (0,0) to (ni,nj)].
  std::vector< vgl_point_2d<double> > polygon_points;

  std::vector< vgl_point_2d<double> > img1_pts;
  img1_pts.push_back( vgl_point_2d<double>( 0, 0 ) );
  img1_pts.push_back( vgl_point_2d<double>( ni, 0 ) );
  img1_pts.push_back( vgl_point_2d<double>( ni, nj ) );
  img1_pts.push_back( vgl_point_2d<double>( 0, nj ) );

  std::vector< vgl_point_2d<double> > img2_pts( 4 );
  for( unsigned i = 0; i < 4; i++ )
  {
    vnl_double_3 hp( img1_pts[i].x(), img1_pts[i].y(), 1.0 );
    vnl_double_3 wp = h * hp;

    if( wp[2] != 0 )
    {
      img2_pts[i] = vgl_point_2d<double>( wp[0] / wp[2] , wp[1] / wp[2] );
    }
    else
    {
      return 0.0;
    }
  }

  // 2. Calculate intersection points between the two images
  for( unsigned i = 0; i < 4; i++ )
  {
    if( does_intersect( img1_pts, img2_pts[i] ) )
    {
      polygon_points.push_back( img2_pts[i] );
    }
    if( does_intersect( img2_pts, img1_pts[i] ) )
    {
      polygon_points.push_back( img1_pts[i] );
    }
  }

  intersecting_points( ni, nj, img2_pts, polygon_points );
  sort_and_remove_duplicates( polygon_points );

  // Early exit case, if only 2 intersection points two frames are
  // co-linear on a single border only.
  if( polygon_points.size() < 3 )
  {
    return 0.0;
  }

  // 3. Compute triangulized area
  double intersection_area = convex_area( polygon_points );
  double frame_area = static_cast<double>( ni * nj );

  // 4. Return area overlap
  return ( frame_area > 0 ? intersection_area / frame_area : 0 );
}


} // end namespace vxl

} // end namespace maptk
