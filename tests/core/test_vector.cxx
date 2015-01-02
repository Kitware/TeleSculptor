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
 * \brief test core vector functionality
 */

#include <test_common.h>

#include <maptk/vector.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(construct_2d)
{
  maptk::vector_2d v2d(10.0, 33.3);
  maptk::vector_2f v2f(5.0f, 4.5f);

  if (v2d.x() != 10.0 || v2f.x() != 5.0f)
  {
    TEST_ERROR("X coordinate of vector_2_ not initialized correctly");
  }

  if (v2d.y() != 33.3 || v2f.y() != 4.5f)
  {
    TEST_ERROR("Y coordinate of vector_2_ not initialized correctly");
  }

}


IMPLEMENT_TEST(construct_3d)
{
  maptk::vector_3d v3d(10.0, 33.3, 12.1);
  maptk::vector_3f v3f(5.0f, 4.5f, -6.3f);

  if (v3d.x() != 10.0 || v3f.x() != 5.0f)
  {
    TEST_ERROR("X coordinate of vector_3_ not initialized correctly");
  }

  if (v3d.y() != 33.3 || v3f.y() != 4.5f)
  {
    TEST_ERROR("Y coordinate of vector_3_ not initialized correctly");
  }

  if (v3d.z() != 12.1 || v3f.z() != -6.3f)
  {
    TEST_ERROR("Z coordinate of vector_3_ not initialized correctly");
  }
}


IMPLEMENT_TEST(construct_4d)
{
  maptk::vector_4d v4d(10.0, 33.3, 12.1, 0.0);
  maptk::vector_4f v4f(5.0f, 4.5f, -6.3f, 100.0f);

  if (v4d.x() != 10.0 || v4f.x() != 5.0f)
  {
    TEST_ERROR("X coordinate of vector_4_ not initialized correctly");
  }

  if (v4d.y() != 33.3 || v4f.y() != 4.5f)
  {
    TEST_ERROR("Y coordinate of vector_4_ not initialized correctly");
  }

  if (v4d.z() != 12.1 || v4f.z() != -6.3f)
  {
    TEST_ERROR("Z coordinate of vector_4_ not initialized correctly");
  }

  if (v4d.w() != 0.0 || v4f.w() != 100.0f)
  {
    TEST_ERROR("W coordinate of vector_4_ not initialized correctly");
  }
}
