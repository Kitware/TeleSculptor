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
 * \brief core ins_data tests
 */

#include <test_common.h>

#include <sstream>
#include <iostream>
#include <maptk/core/ins_data.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

static bool test_iostream(const maptk::ins_data& d)
{
  std::cout << "Sample INS data:\n" << d << std::flush;
  std::stringstream ss;
  ss << d;

  // read the data back in
  maptk::ins_data d2;
  ss >> d2;
  std::cout << "Re-read data as:\n" << d2;

  return d == d2;
}


IMPLEMENT_TEST(iostream_low_prec)
{
  maptk::ins_data d(28.2, -37.1, 0.034,
                    47.9, 82.0, 170.0,
                    "TEST",
                    23.0, 345,
                    0.0, 2.5, 3.0,
                    3, 8, -1);
  if( !test_iostream(d) )
  {
    TEST_ERROR("Written INS data does not match what is read back in.");
  }
}


IMPLEMENT_TEST(iostream_high_prec)
{
  maptk::ins_data d(28.29342, -37.19432, 0.00000349,
                    47.9347286349, 82.033292098, 1700.0,
                    "MAPTK",
                    23923423.003, 345,
                    0.0, 2.5, 3.0,
                    3, 8, -1);
  if( !test_iostream(d) )
  {
    TEST_ERROR("Written INS data does not match what is read back in.");
  }
}
