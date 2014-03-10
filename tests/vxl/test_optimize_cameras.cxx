/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <iostream>

#include <test_common.h>

#include <maptk/core/camera_map.h>
#include <maptk/core/landmark_map.h>
#include <maptk/core/track_set.h>
#include <maptk/vxl/optimize_cameras.h>


#define TEST_ARGS ()
DECLARE_TEST_MAP();

int main(int argc, char* argv[])
{
  CHECK_ARGS(1);
  maptk::vxl::optimize_cameras::register_self();
  testname_t const testname = argv[1];
  RUN_TEST(testname);
}


IMPLEMENT_TEST(creation)
{
  using namespace maptk;
  algo::optimize_cameras_sptr cam_optimizer = algo::optimize_cameras::create("vxl");
  if (!cam_optimizer)
  {
    TEST_ERROR("Unable to create vxl::optimize_cameras by impl name.");
  }
}


IMPLEMENT_TEST(uninitialized)
{
  using namespace maptk;
  using namespace std;

  camera_map_sptr cam_map;
  landmark_map_sptr lm_map;
  track_set_sptr trk_set;

  vxl::optimize_cameras optimizer;

  cerr << "cam_map before: " << cam_map << endl;

  optimizer.optimize(cam_map, trk_set, lm_map);

  cerr << "cam_map after: " << cam_map << endl;

  TEST_EQUAL("cam_map", cam_map, 0);
}


IMPLEMENT_TEST(empty_input)
{
  using namespace maptk;
  using namespace std;

  camera_map_sptr cam_map(new simple_camera_map());
  landmark_map_sptr lm_map(new simple_landmark_map());
  track_set_sptr trk_set(new simple_track_set());

  vxl::optimize_cameras optimizer;

  camera_map_sptr orig_map = cam_map;

  cerr << "cam_map before: " << cam_map << endl;
  optimizer.optimize(cam_map, trk_set, lm_map);
  cerr << "cam_map after : " << cam_map << endl;
  cerr << "orig map      : " << orig_map << endl;

  // make sure that a new camera map was created, but nothing was put in it.
  TEST_EQUAL("cam_map reference",
      cam_map == orig_map,
      false);
  TEST_EQUAL("cam_map size", cam_map->size(), 0);
  TEST_EQUAL("orig map size", orig_map->size(), 0);
}
