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


inline
maptk::vector_3d random_point3(double stdev)
{
  maptk::vector_3d v(rng.normal64(), rng.normal64(), rng.normal64());
  return stdev * v;
}

inline
maptk::vector_2d random_point2(double stdev)
{
  maptk::vector_2d v(rng.normal64(), rng.normal64());
  return stdev * v;
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


// add Gaussian noise to the landmark positions
maptk::landmark_map_sptr
noisy_landmarks(maptk::landmark_map_sptr landmarks,
                double stdev=1.0)
{
  using namespace maptk;

  landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type& p, lm_map)
  {
    landmark_d& lm = dynamic_cast<landmark_d&>(*p.second);
    lm.set_loc(lm.get_loc() + random_point3(stdev));
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


// add positional and rotational Gaussian noise to cameras
maptk::camera_map_sptr
noisy_cameras(maptk::camera_map_sptr cameras,
              double pos_stdev=1.0, double rot_stdev=1.0)
{
  using namespace maptk;

  camera_map::map_camera_t cam_map = cameras->cameras();
  BOOST_FOREACH(camera_map::map_camera_t::value_type& p, cam_map)
  {
    camera_d& cam = dynamic_cast<camera_d&>(*p.second);
    cam.set_center(cam.get_center() + random_point3(pos_stdev));
    rotation_d rand_rot(random_point3(rot_stdev));
    cam.set_rotation(cam.get_rotation() * rand_rot);
  }
  return camera_map_sptr(new simple_camera_map(cam_map));
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


// randomly drop a fraction of the track states
maptk::track_set_sptr
subset_tracks(maptk::track_set_sptr in_tracks, double keep_frac=0.75)
{
  using namespace maptk;

  std::srand(0);
  std::vector<track_sptr> tracks = in_tracks->tracks();
  std::vector<track_sptr> new_tracks;
  const int rand_thresh = static_cast<int>(keep_frac * RAND_MAX);
  BOOST_FOREACH(const track_sptr& t, tracks)
  {
    track_sptr nt(new track);
    nt->set_id(t->id());
    std::cout << "track "<<t->id()<<":";
    for(track::history_const_itr it=t->begin(); it!=t->end(); ++it)
    {
      if(std::rand() < rand_thresh)
      {
        nt->append(*it);
        std::cout << " .";
      }
      else
      {
        std::cout << " X";
      }
    }
    std::cout << std::endl;
    new_tracks.push_back(nt);
  }
  return track_set_sptr(new simple_track_set(new_tracks));
}


// add Gaussian noise to track feature locations
maptk::track_set_sptr
noisy_tracks(maptk::track_set_sptr in_tracks, double stdev=1.0)
{
  using namespace maptk;

  std::vector<track_sptr> tracks = in_tracks->tracks();
  std::vector<track_sptr> new_tracks;
  BOOST_FOREACH(const track_sptr& t, tracks)
  {
    track_sptr nt(new track);
    nt->set_id(t->id());
    for(track::history_const_itr it=t->begin(); it!=t->end(); ++it)
    {
      vector_2d loc = it->feat->loc() + random_point2(stdev);
      track::track_state ts(*it);
      ts.feat = feature_sptr(new feature_d(loc));
      nt->append(ts);
    }
    new_tracks.push_back(nt);
  }
  return track_set_sptr(new simple_track_set(new_tracks));
}


// input to SBA is the ideal solution, make sure it doesn't diverge
IMPLEMENT_TEST(from_solution)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

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


// add noise to landmarks before input to SBA
IMPLEMENT_TEST(noisy_landmarks)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = noisy_landmarks(landmarks, 0.1);


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
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
IMPLEMENT_TEST(noisy_landmarks_noisy_cameras)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = noisy_cameras(cameras, 0.1, 0.1);


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
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// initialize all landmarks to the origin as input to SBA
IMPLEMENT_TEST(zero_landmarks)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

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
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// initialize all landmarks to the origin and all cameras to same location as input to SBA
IMPLEMENT_TEST(zero_landmarks_same_cameras)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

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
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
// select a subset of cameras to optimize
IMPLEMENT_TEST(subset_cameras)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = noisy_cameras(cameras, 0.1, 0.1);

  camera_map::map_camera_t cam_map = cameras0->cameras();
  camera_map::map_camera_t cam_map2;
  BOOST_FOREACH(camera_map::map_camera_t::value_type& p, cam_map)
  {
    /// take every third camera
    if(p.first % 3 == 0)
    {
      cam_map2.insert(p);
    }
  }
  cameras0 = camera_map_sptr(new simple_camera_map(cam_map2));


  TEST_EQUAL("Reduced number of cameras", cameras0->size(), 7);

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
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
// select a subset of landmarks to optimize
IMPLEMENT_TEST(subset_landmarks)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = noisy_cameras(cameras, 0.1, 0.1);

  // remove some landmarks
  landmark_map::map_landmark_t lm_map = landmarks0->landmarks();
  lm_map.erase(1);
  lm_map.erase(4);
  lm_map.erase(5);
  landmarks0 = landmark_map_sptr(new simple_landmark_map(lm_map));

  TEST_EQUAL("Reduced number of landmarks", landmarks0->size(), 5);

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
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
// select a subset of tracks/track_states to constrain the problem
IMPLEMENT_TEST(subset_tracks)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = noisy_cameras(cameras, 0.1, 0.1);

  // remove some tracks/track_states
  track_set_sptr tracks0 = subset_tracks(tracks, 0.5);


  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks0->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks0);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks0->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras and tracks before input to SBA
// select a subset of tracks/track_states to constrain the problem
IMPLEMENT_TEST(noisy_tracks)
{
  using namespace maptk;
  vxl::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("g_tolerance", "1e-12");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = noisy_cameras(cameras, 0.1, 0.1);

  // remove some tracks/track_states and add Gaussian noise
  const double track_stdev = 1.0;
  track_set_sptr tracks0 = noisy_tracks(subset_tracks(tracks, 0.5),
                                        track_stdev);


  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks0->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks0);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks0->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, track_stdev);
}
