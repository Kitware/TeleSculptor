/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief test core track class
 */

#include <test_common.h>

#include <iostream>
#include <vector>
#include <maptk/core/track.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(track_id)
{
  using namespace maptk;
  track t;
  TEST_EQUAL("Initial Track ID", t.id(), 0);

  t.set_id(25);
  TEST_EQUAL("Get/Set Track ID", t.id(), 25);
}
