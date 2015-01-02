/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief test OCV match set class
 */

#include <test_common.h>

#include <maptk/plugins/ocv/match_set.h>



#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(default_set)
{
  using namespace maptk;
  ocv::match_set ms;
  if (ms.size() != 0)
  {
    TEST_ERROR("Default match_set is not empty");
  }
  if (!ms.matches().empty())
  {
    TEST_ERROR("Default match_set produces non-empty features");
  }
}


// It seems operator== on cv::DMatch is not defined in OpenCV
static bool dmatch_equal(const cv::DMatch& dm1, const cv::DMatch& dm2)
{
  return dm1.queryIdx == dm2.queryIdx &&
         dm1.trainIdx == dm2.trainIdx &&
         dm1.imgIdx == dm2.imgIdx &&
         dm1.distance == dm2.distance;
}


IMPLEMENT_TEST(populated_set)
{
  using namespace maptk;
  const unsigned num_matches = 100;
  std::vector<cv::DMatch> dms;
  for (unsigned i=0; i<num_matches; ++i)
  {
    cv::DMatch dm(i, num_matches-i-1, FLT_MAX);
    dms.push_back(dm);
  }
  ocv::match_set ms(dms);
  if (ms.size() != num_matches)
  {
    TEST_ERROR("match_set is not the expected size");
  }
  std::vector<cv::DMatch> dms2 = ms.ocv_matches();
  if (!std::equal(dms.begin(), dms.end(), dms2.begin(), dmatch_equal))
  {
    TEST_ERROR("match_set does not contain original cv::DMatch");
  }
  std::vector<match> mats = ms.matches();
  if (mats.size() != ms.size())
  {
    TEST_ERROR("match_set does not produce expected number of matches");
  }

  simple_match_set simp_ms(mats);
  dms2 = ocv::matches_to_ocv_dmatch(simp_ms);
  if (!std::equal(dms.begin(), dms.end(), dms2.begin(), dmatch_equal))
  {
    TEST_ERROR("conversion to and from MAPTK matches does not preserve cv::DMatch");
  }
}
