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


IMPLEMENT_TEST(cube)
{
  using namespace maptk;
  vxl::bundle_adjust ba;

  // create landmarks at the corners of a cube
  std::vector<vector_3d> landmark_pts;
  landmark_pts.push_back(vector_3d(-1, -1, -1));
  landmark_pts.push_back(vector_3d(-1, -1,  1));
  landmark_pts.push_back(vector_3d(-1,  1, -1));
  landmark_pts.push_back(vector_3d(-1,  1,  1));
  landmark_pts.push_back(vector_3d( 1, -1, -1));
  landmark_pts.push_back(vector_3d( 1, -1,  1));
  landmark_pts.push_back(vector_3d( 1,  1, -1));
  landmark_pts.push_back(vector_3d( 1,  1,  1));
  const landmark_id_t num_pts = static_cast<landmark_id_t>(landmark_pts.size());

  // create a camera sequence (elliptical path)
  // and a the projections of each landmark into each frame
  camera_intrinsics_d K(1000, vector_2d(640,480));
  rotation_d R; // identity
  camera_map::map_camera_t cameras;
  std::vector<std::vector<vector_2d> > projections(landmark_pts.size());
  const frame_id_t num_cams = 20;
  for (frame_id_t i=0; i<num_cams; ++i)
  {
    double frac = static_cast<double>(i) / num_cams;
    double x = 4 * std::cos(2*frac);
    double y = 3 * std::sin(2*frac);
    camera_d* cam = new camera_d(vector_3d(x,y,2+frac), R, K);
    // look at the origin
    cam->look_at(vector_3d(0,0,0));
    cameras[i] = camera_sptr(cam);
    //std::cout << "camera "<<i<<" center "<<cam->get_center()<<std::endl;
    for (landmark_id_t j=0; j<num_pts; ++j)
    {
      projections[j].push_back(cam->project(landmark_pts[j]));
      //std::cout << "camera "<<i<<" point "<<j<<" : "<<projections[j].back() <<std::endl;
    }
  }

  // create tracks from the projections
  std::vector<track_sptr> tracks;
  for (landmark_id_t i=0; i<num_pts; ++i)
  {
    track_sptr t(new track);
    t->set_id(i);
    tracks.push_back(t);
    for (frame_id_t j=0; j<num_cams; ++j)
    {
      feature_sptr f(new feature_d(projections[i][j]));
      t->append(track::track_state(j, f, descriptor_sptr()));
    }
  }

  // initialize all landmarks to the origin
  landmark_map::map_landmark_t landmarks;
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
