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
  rng.reseed(1234);

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


// construct a map of landmarks at the corners of a cube centered at c
// with a side length of s
maptk::landmark_map_sptr
cube_corners(double s, const maptk::vector_3d& c=maptk::vector_3d(0,0,0))
{
  using namespace maptk;

  // create corners of a cube
  landmark_map::map_landmark_t landmarks;
  s /= 2.0;
  landmarks[0] = landmark_sptr(new landmark_d(c + vector_3d(-s, -s, -s)));
  landmarks[1] = landmark_sptr(new landmark_d(c + vector_3d(-s, -s,  s)));
  landmarks[2] = landmark_sptr(new landmark_d(c + vector_3d(-s,  s, -s)));
  landmarks[3] = landmark_sptr(new landmark_d(c + vector_3d(-s,  s,  s)));
  landmarks[4] = landmark_sptr(new landmark_d(c + vector_3d( s, -s, -s)));
  landmarks[5] = landmark_sptr(new landmark_d(c + vector_3d( s, -s,  s)));
  landmarks[6] = landmark_sptr(new landmark_d(c + vector_3d( s,  s, -s)));
  landmarks[7] = landmark_sptr(new landmark_d(c + vector_3d( s,  s,  s)));

  return landmark_map_sptr(new simple_landmark_map(landmarks));
}


// construct map of landmarks will all locations at c
maptk::landmark_map_sptr
init_landmarks(maptk::landmark_id_t num_lm,
               const maptk::vector_3d& c=maptk::vector_3d(0,0,0))
{
  using namespace maptk;

  landmark_map::map_landmark_t lm_map;
  for (landmark_id_t i=0; i<num_lm; ++i)
  {
    lm_map[i] = landmark_sptr(new landmark_d(c));
  }
  return landmark_map_sptr(new simple_landmark_map(lm_map));
}


// create a camera sequence (elliptical path)
maptk::camera_map_sptr
camera_seq(maptk::frame_id_t num_cams = 20)
{
  using namespace maptk;
  camera_map::map_camera_t cameras;

  // create a camera sequence (elliptical path)
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
  return camera_map_sptr(new simple_camera_map(cameras));
}


// create an initial camera sequence with all cameras at the same location
maptk::camera_map_sptr
init_cameras(maptk::frame_id_t num_cams = 20)
{
  using namespace maptk;
  camera_map::map_camera_t cameras;

  // create a camera sequence (elliptical path)
  camera_intrinsics_d K(1000, vector_2d(640,480));
  rotation_d R; // identity
  vector_3d c(0, 0, 1);
  for (frame_id_t i=0; i<num_cams; ++i)
  {
    camera_d* cam = new camera_d(c, R, K);
    // look at the origin
    cam->look_at(vector_3d(0,0,0), vector_3d(0,1,0));
    cameras[i] = camera_sptr(cam);
  }
  return camera_map_sptr(new simple_camera_map(cameras));
}


// create tracks by projecting the landmarks into the cameras
maptk::track_set_sptr
projected_tracks(maptk::landmark_map_sptr landmarks,
                 maptk::camera_map_sptr cameras)
{
  using namespace maptk;
  std::vector<track_sptr> tracks;
  camera_map::map_camera_t cam_map = cameras->cameras();
  landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  const track_id_t num_pts = static_cast<track_id_t>(landmarks->size());
  for (track_id_t i=0; i<num_pts; ++i)
  {
    track_sptr t(new track);
    t->set_id(i);
    tracks.push_back(t);
    BOOST_FOREACH(const camera_map::map_camera_t::value_type& p, cam_map)
    {
      const camera_d& cam = dynamic_cast<const camera_d&>(*p.second);
      feature_sptr f(new feature_d(cam.project(lm_map[i]->loc())));
      t->append(track::track_state(p.first, f, descriptor_sptr()));
    }
  }
  return track_set_sptr(new simple_track_set(tracks));
}


// input to SBA is the ideal solution, make sure it doesn't diverge
IMPLEMENT_TEST(from_solution)
{
  using namespace maptk;
  vxl::bundle_adjust ba;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse > 1e-12)
  {
    TEST_ERROR("Initial reprojection RMSE should be small");
  }

  ba.optimize(cameras, landmarks, tracks);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-12);
}


// initialize all landmarks to the origin as input to SBA
IMPLEMENT_TEST(zero_landmarks)
{
  using namespace maptk;
  vxl::bundle_adjust ba;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_map_sptr landmarks0 = init_landmarks(landmarks->size());


  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-6);
}


// initialize all landmarks to the origin and all cameras to same location as input to SBA
IMPLEMENT_TEST(zero_landmarks_same_cameras)
{
  using namespace maptk;
  vxl::bundle_adjust ba;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_map_sptr landmarks0 = init_landmarks(landmarks->size());

  // initialize all cameras to at (0,0,1) looking at the origin
  camera_map_sptr cameras0 = init_cameras(cameras->size());


  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-6);
}
