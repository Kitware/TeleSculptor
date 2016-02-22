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

#include <test_common.h>
#include <test_scene.h>
#include <algo/test_triangulate_landmarks.h>

#include <maptk/plugins/core/register_algorithms.h>
#include <maptk/plugins/core/triangulate_landmarks.h>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  kwiver::maptk::core::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(create)
{
  using namespace kwiver::vital;
  algo::triangulate_landmarks_sptr tri_lm = algo::triangulate_landmarks::create("core");
  if (!tri_lm)
  {
    TEST_ERROR("Unable to create core::triangulate_landmarks by name");
  }
}


// input to triangulation is the ideal solution, make sure it doesn't diverge
IMPLEMENT_TEST(from_solution)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::maptk::testing::test_from_solution(tri_lm);
}


// input to triangulation is the ideal solution, make sure it doesn't diverge
IMPLEMENT_TEST(from_solution_homog)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::vital::config_block_sptr cfg = tri_lm.get_configuration();
  cfg->set_value("homogeneous", "true");
  tri_lm.set_configuration(cfg);
  kwiver::maptk::testing::test_from_solution(tri_lm);
}


// add noise to landmarks before input to triangulation
IMPLEMENT_TEST(noisy_landmarks)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::maptk::testing::test_noisy_landmarks(tri_lm);
}


// add noise to landmarks before input to triangulation
IMPLEMENT_TEST(noisy_landmarks_homog)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::vital::config_block_sptr cfg = tri_lm.get_configuration();
  cfg->set_value("homogeneous", "true");
  tri_lm.set_configuration(cfg);
  kwiver::maptk::testing::test_noisy_landmarks(tri_lm);
}


// initialize all landmarks to the origin as input to triangulation
IMPLEMENT_TEST(zero_landmarks)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::maptk::testing::test_zero_landmarks(tri_lm);
}


// initialize all landmarks to the origin as input to triangulation
IMPLEMENT_TEST(zero_landmarks_homog)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::vital::config_block_sptr cfg = tri_lm.get_configuration();
  cfg->set_value("homogeneous", "true");
  tri_lm.set_configuration(cfg);
  kwiver::maptk::testing::test_zero_landmarks(tri_lm);
}


// select a subset of cameras to triangulation from
IMPLEMENT_TEST(subset_cameras)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::maptk::testing::test_subset_cameras(tri_lm);
}


// select a subset of cameras to triangulation from
IMPLEMENT_TEST(subset_cameras_homog)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::vital::config_block_sptr cfg = tri_lm.get_configuration();
  cfg->set_value("homogeneous", "true");
  tri_lm.set_configuration(cfg);
  kwiver::maptk::testing::test_subset_cameras(tri_lm);
}


// select a subset of landmarks to triangulate
IMPLEMENT_TEST(subset_landmarks)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::maptk::testing::test_subset_landmarks(tri_lm);
}


// select a subset of landmarks to triangulate
IMPLEMENT_TEST(subset_landmarks_homog)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::vital::config_block_sptr cfg = tri_lm.get_configuration();
  cfg->set_value("homogeneous", "true");
  tri_lm.set_configuration(cfg);
  kwiver::maptk::testing::test_subset_landmarks(tri_lm);
}


// select a subset of tracks/track_states to constrain the problem
IMPLEMENT_TEST(subset_tracks)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::maptk::testing::test_subset_tracks(tri_lm);
}


// select a subset of tracks/track_states to constrain the problem
IMPLEMENT_TEST(subset_tracks_homog)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::vital::config_block_sptr cfg = tri_lm.get_configuration();
  cfg->set_value("homogeneous", "true");
  tri_lm.set_configuration(cfg);
  kwiver::maptk::testing::test_subset_tracks(tri_lm);
}


// select a subset of tracks/track_states and add noise
IMPLEMENT_TEST(noisy_tracks)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::maptk::testing::test_noisy_tracks(tri_lm);
}

// select a subset of tracks/track_states and add noise
IMPLEMENT_TEST(noisy_tracks_homog)
{
  kwiver::maptk::core::triangulate_landmarks tri_lm;
  kwiver::vital::config_block_sptr cfg = tri_lm.get_configuration();
  cfg->set_value("homogeneous", "true");
  tri_lm.set_configuration(cfg);
  kwiver::maptk::testing::test_noisy_tracks(tri_lm);
}
