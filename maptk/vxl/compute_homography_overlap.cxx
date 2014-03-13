/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of homography overlap helper functions
 */

#include "compute_homography_overlap.h"

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_convex.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vgl/vgl_area.h>
#include <vgl/vgl_intersection.h>

#include <vnl/vnl_double_3.h>

#include <algorithm>
#include <vector>

namespace maptk
{

namespace vxl
{



bool does_intersect( const std::vector< vgl_point_2d<double> >& poly, const vgl_point_2d<double>& pt )
{
  vgl_polygon<double> pol( &poly[0], poly.size() );
  return pol.contains( pt );
}

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

double convex_area( const std::vector< vgl_point_2d<double> >& poly )
{
  return vgl_area( vgl_convex_hull( poly ) );
}

bool compare_poly_pts( const vgl_point_2d<double>& p1, const vgl_point_2d<double>& p2 )
{
  if( p1.x() < p2.x() )
    return true;
  else if( p1.x() > p2.x() )
    return false;

  return ( p1.y() < p2.y() );
}

void remove_duplicates( std::vector< vgl_point_2d<double> >& poly )
{
  if( poly.size() == 0 )
    return;

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


double
overlap( const vnl_double_3x3 h, const unsigned ni, const unsigned nj )
{
  std::vector< vgl_point_2d<double> > polygon_points;

  // 1. Warp corner points to current image
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

  // 2. Calculate intersection points of the input image with each other
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

  remove_duplicates( polygon_points );

  if( polygon_points.size() < 3 )
  {
    return 0.0;
  }

  // 3. Compute triangulized area
  double intersection_area = convex_area( polygon_points );
  double frame_area = ni * nj;

  // 4. Return area overlap
  return ( frame_area > 0 ? intersection_area / frame_area : 0 );
}


} // end namespace vxl

} // end namespace maptk
