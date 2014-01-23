/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/metrics.h>
#include <maptk/vxl/register.h>
#include <maptk/vxl/bundle_adjust.h>
#include <vnl/vnl_random.h>
#include <boost/foreach.hpp>

static vnl_random rng;


maptk::vector_2d random_point(double mean, double stdev)
{
  maptk::vector_2d v(rng.normal64(), rng.normal64());
  return stdev * v + mean;
}

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  maptk::vxl::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(create)
{
  using namespace maptk;
  algo::bundle_adjust_sptr ba = algo::bundle_adjust::create("vxl");
  if (!ba)
  {
    TEST_ERROR("Unable to create vxl::bundle_adjust by name");
  }
}


// construct a vector of points at the corners of a cube centered at c
// with a side length of s
std::vector<maptk::vector_3d>
cube_corners(double s, const maptk::vector_3d& c=maptk::vector_3d(0,0,0))
{
  using namespace maptk;

  // create corners of a cube
  std::vector<vector_3d> corners;
  s /= 2.0;
  corners.push_back(c + vector_3d(-s, -s, -s));
  corners.push_back(c + vector_3d(-s, -s,  s));
  corners.push_back(c + vector_3d(-s,  s, -s));
  corners.push_back(c + vector_3d(-s,  s,  s));
  corners.push_back(c + vector_3d( s, -s, -s));
  corners.push_back(c + vector_3d( s, -s,  s));
  corners.push_back(c + vector_3d( s,  s, -s));
  corners.push_back(c + vector_3d( s,  s,  s));

  return corners;
}


// create a camera sequence (elliptical path)
maptk::camera_map::map_camera_t
camera_seq(frame_id_t num_cams = 20)
{
  using namespace maptk;
  camera_map::map_camera_t cameras;

  // create a camera sequence (elliptical path)
  // and a the projections of each landmark into each frame
  camera_intrinsics_d K(1000, vector_2d(640,480));
  rotation_d R; // identity
  for (frame_id_t i=0; i<num_cams; ++i)
  {
    double frac = static_cast<double>(i) / num_cams;
    double x = 4 * std::cos(2*frac);
    double y = 3 * std::sin(2*frac);
    camera_d* cam = new camera_d(vector_3d(x,y,2+frac), R, K);
    // look at the origin
    cam->look_at(vector_3d(0,0,0));
    cameras[i] = camera_sptr(cam);
  }
  return cameras;
}


// create tracks by projecting the points into the cameras
std::vector<maptk::track_sptr>
projected_tracks(const std::vector<maptk::vector_3d>& points,
                 const maptk::camera_map::map_camera_t& cameras)
{
  using namespace maptk;
  std::vector<track_sptr> tracks;
  const track_id_t num_pts = static_cast<track_id_t>(points.size());
  for (track_id_t i=0; i<num_pts; ++i)
  {
    track_sptr t(new track);
    t->set_id(i);
    tracks.push_back(t);
    BOOST_FOREACH(const camera_map::map_camera_t::value_type& p, cameras)
    {
      const camera_d& cam = dynamic_cast<const camera_d&>(*p.second);
      feature_sptr f(new feature_d(cam.project(points[i])));
      t->append(track::track_state(p.first, f, descriptor_sptr()));
    }
  }
  return tracks;
}


IMPLEMENT_TEST(cube)
{
  using namespace maptk;
  vxl::bundle_adjust ba;

  // create landmarks at the corners of a cube
  std::vector<vector_3d> points = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map::map_camera_t cameras = camera_seq();

  // create tracks from the projections
  std::vector<track_sptr> tracks = projected_tracks(points, cameras);

  // initialize all landmarks to the origin
  landmark_map::map_landmark_t landmarks;
  const landmark_id_t num_pts = static_cast<landmark_id_t>(points.size());
  for (landmark_id_t i=0; i<num_pts; ++i)
  {
    landmarks[i] = landmark_sptr(new landmark_d(vector_3d(0,0,0)));
  }

  camera_map_sptr cam_map(new simple_camera_map(cameras));
  landmark_map_sptr lm_map(new simple_landmark_map(landmarks));
  track_set_sptr trk_set(new simple_track_set(tracks));

  double init_rmse = reprojection_rmse(cam_map->cameras(),
                                       lm_map->landmarks(),
                                       trk_set->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cam_map, lm_map, trk_set);

  double end_rmse = reprojection_rmse(cam_map->cameras(),
                                      lm_map->landmarks(),
                                      trk_set->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-6);
}
