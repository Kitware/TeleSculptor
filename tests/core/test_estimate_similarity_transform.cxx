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

#include <test_common.h>

#include <vector>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/estimate_similarity_transform.h>
#include <maptk/core/camera.h>
#include <maptk/core/camera_map.h>
#include <maptk/core/config_block.h>
#include <maptk/core/landmark.h>
#include <maptk/core/landmark_map.h>
#include <maptk/core/similarity.h>
#include <maptk/core/types.h>
#include <maptk/core/vector.h>


#define TEST_ARGS ()
DECLARE_TEST_MAP();


namespace
{

/// Dummy algo impl to test function wrappers
class dummy_est
  : public maptk::algo::algorithm_impl<dummy_est, maptk::algo::estimate_similarity_transform>
{
public:
  dummy_est()
    : expected_size(0)
  {}

  dummy_est(size_t expected_size)
    : expected_size(expected_size)
  {}

  virtual ~dummy_est() {}

  std::string impl_name() const { return "dummy_est"; }

  void set_configuration(maptk::config_block_sptr config) {}
  bool check_configuration(maptk::config_block_sptr config) const {return true;}

  maptk::similarity_d
  estimate_transform(std::vector<maptk::vector_3d> const& from,
                     std::vector<maptk::vector_3d> const& to) const
  {
    TEST_EQUAL("input vector length equality", from.size() == to.size(), true);
    TEST_EQUAL("from vector size", from.size(), expected_size);
    TEST_EQUAL("to vector size", to.size(), expected_size);

    return maptk::similarity_d();
  }

  size_t expected_size;
};

}


int
main(int argc, char *argv[])
{
  CHECK_ARGS(1);
  testname_t const testname = argv[1];
  RUN_TEST(testname);
}


using namespace maptk;
using namespace std;


IMPLEMENT_TEST(baseline)
{
  algo::estimate_similarity_transform_sptr est(new dummy_est());
  vector<vector_3d> pts1, pts2;
  est->estimate_transform(pts1, pts2);
}


IMPLEMENT_TEST(vector_of_cameras)
{
  size_t N(100);
  vector<camera_sptr> from_cams, to_cams;
  for(size_t i=0; i<N; ++i)
  {
    from_cams.push_back(camera_sptr(new camera_d()));
    to_cams.push_back(camera_sptr(new camera_d()));
  }

  algo::estimate_similarity_transform_sptr est(new dummy_est(N));
  est->estimate_transform(from_cams, to_cams);
}


IMPLEMENT_TEST(vector_of_landmarks)
{
  size_t N(73);
  vector<landmark_sptr> from_lmks, to_lmks;
  for(size_t i=0; i<N; ++i)
  {
    from_lmks.push_back(landmark_sptr(new landmark_d()));
    to_lmks.push_back(landmark_sptr(new landmark_d()));
  }

  algo::estimate_similarity_transform_sptr est(new dummy_est(N));
  est->estimate_transform(from_lmks, to_lmks);
}


IMPLEMENT_TEST(sync_camera_map)
{
  size_t N(63);

  camera_map::map_camera_t from_map, to_map;
  for(frame_id_t i=0; i<static_cast<frame_id_t>(N); ++i)
  {
    from_map[i] = camera_sptr(new camera_d());
    to_map[i] = camera_sptr(new camera_d());
  }
  camera_map_sptr from_cmap(new simple_camera_map(from_map)),
                  to_cmap(new simple_camera_map(to_map));

  algo::estimate_similarity_transform_sptr est(new dummy_est(N));
  est->estimate_transform(from_cmap, to_cmap);
}


IMPLEMENT_TEST(disjoint_camera_maps)
{
  // uniform overlap
  frame_id_t i_b=0, i_e=50,
             j_b=25, j_e=75;
  size_t overlap=static_cast<size_t>(i_e - j_b);

  camera_map::map_camera_t from_map, to_map;
  for(frame_id_t i=i_b; i<i_e; ++i)
  {
    from_map[i] = camera_sptr(new camera_d());
  }
  for(frame_id_t j=j_b; j<j_e; ++j)
  {
    to_map[j] = camera_sptr(new camera_d());
  }
  camera_map_sptr from_cmap(new simple_camera_map(from_map)),
                  to_cmap(new simple_camera_map(to_map));

  algo::estimate_similarity_transform_sptr est1(new dummy_est(overlap));
  est1->estimate_transform(from_cmap, to_cmap);


  // disjoint overlap
  from_map = camera_map::map_camera_t();
  to_map = camera_map::map_camera_t();
  from_map[0] = camera_sptr(new camera_d());
  from_map[1] = camera_sptr(new camera_d());
  from_map[3] = camera_sptr(new camera_d());
  from_map[4] = camera_sptr(new camera_d());
  from_map[5] = camera_sptr(new camera_d());
  from_map[9] = camera_sptr(new camera_d());
  to_map  [1] = camera_sptr(new camera_d());
  to_map  [2] = camera_sptr(new camera_d());
  to_map  [3] = camera_sptr(new camera_d());
  to_map  [5] = camera_sptr(new camera_d());
  to_map  [6] = camera_sptr(new camera_d());
  to_map  [9] = camera_sptr(new camera_d());
  to_map [11] = camera_sptr(new camera_d());
  to_map [94] = camera_sptr(new camera_d());
  camera_map_sptr from_cmap2(new simple_camera_map(from_map)),
                  to_cmap2(new simple_camera_map(to_map));

  algo::estimate_similarity_transform_sptr est2(new dummy_est(4));
  est2->estimate_transform(from_cmap2, to_cmap2);
}


IMPLEMENT_TEST(sync_landmark_map)
{
  size_t N(63);

  landmark_map::map_landmark_t from_map, to_map;
  for(frame_id_t i=0; i<static_cast<frame_id_t>(N); ++i)
  {
    from_map[i] = landmark_sptr(new landmark_d());
    to_map[i] = landmark_sptr(new landmark_d());
  }
  landmark_map_sptr from_cmap(new simple_landmark_map(from_map)),
                    to_cmap(new simple_landmark_map(to_map));

  algo::estimate_similarity_transform_sptr est(new dummy_est(N));
  est->estimate_transform(from_cmap, to_cmap);
}


IMPLEMENT_TEST(disjoint_landmark_maps)
{
  // uniform overlap
  frame_id_t i_b=0, i_e=50,
             j_b=25, j_e=75;
  size_t overlap=static_cast<size_t>(i_e - j_b);

  landmark_map::map_landmark_t from_map, to_map;
  for(frame_id_t i=i_b; i<i_e; ++i)
  {
    from_map[i] = landmark_sptr(new landmark_d());
  }
  for(frame_id_t j=j_b; j<j_e; ++j)
  {
    to_map[j] = landmark_sptr(new landmark_d());
  }
  landmark_map_sptr from_cmap(new simple_landmark_map(from_map)),
                    to_cmap(new simple_landmark_map(to_map));

  algo::estimate_similarity_transform_sptr est1(new dummy_est(overlap));
  est1->estimate_transform(from_cmap, to_cmap);


  // disjoint overlap
  from_map = landmark_map::map_landmark_t();
  to_map = landmark_map::map_landmark_t();
  from_map[0] = landmark_sptr(new landmark_d());
  from_map[1] = landmark_sptr(new landmark_d());
  from_map[3] = landmark_sptr(new landmark_d());
  from_map[4] = landmark_sptr(new landmark_d());
  from_map[5] = landmark_sptr(new landmark_d());
  from_map[9] = landmark_sptr(new landmark_d());
  to_map  [1] = landmark_sptr(new landmark_d());
  to_map  [2] = landmark_sptr(new landmark_d());
  to_map  [3] = landmark_sptr(new landmark_d());
  to_map  [5] = landmark_sptr(new landmark_d());
  to_map  [6] = landmark_sptr(new landmark_d());
  to_map  [9] = landmark_sptr(new landmark_d());
  to_map [11] = landmark_sptr(new landmark_d());
  to_map [94] = landmark_sptr(new landmark_d());
  landmark_map_sptr from_cmap2(new simple_landmark_map(from_map)),
                    to_cmap2(new simple_landmark_map(to_map));

  algo::estimate_similarity_transform_sptr est2(new dummy_est(4));
  est2->estimate_transform(from_cmap2, to_cmap2);
}
