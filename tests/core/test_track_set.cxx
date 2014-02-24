/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <iostream>
#include <vector>

#include <maptk/core/track.h>
#include <maptk/core/track_set.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(accessor_functions)
{
  using namespace maptk;

  unsigned track_id = 0;

  std::vector< track_sptr > test_tracks;

  track::track_state test_state1( 1, feature_sptr(), descriptor_sptr() );
  track::track_state test_state2( 2, feature_sptr(), descriptor_sptr() );
  track::track_state test_state3( 3, feature_sptr(), descriptor_sptr() );

  test_tracks.push_back( track_sptr( new track( test_state1 ) ) );
  test_tracks.back()->set_id( track_id++ );
  test_tracks.push_back( track_sptr( new track( test_state1 ) ) );
  test_tracks.back()->set_id( track_id++ );
  test_tracks.push_back( track_sptr( new track( test_state2 ) ) );
  test_tracks.back()->set_id( track_id++ );
  test_tracks.push_back( track_sptr( new track( test_state3 ) ) );
  test_tracks.back()->set_id( track_id++ );

  test_tracks[0]->append( test_state2 );
  test_tracks[0]->append( test_state3 );
  test_tracks[1]->append( test_state2 );
  test_tracks[2]->append( test_state3 );

  track_set_sptr test_set( new simple_track_set( test_tracks ) );

  TEST_EQUAL("Total set size", test_set->size(), 4);

  TEST_EQUAL("Active set size 1", test_set->active_tracks(-1)->size(), 3);
  TEST_EQUAL("Active set size 2", test_set->active_tracks(-2)->size(), 3);
  TEST_EQUAL("Active set size 3", test_set->active_tracks(-3)->size(), 2);

  TEST_EQUAL("Terminated set size", test_set->terminated_tracks(-1)->size(), 3);
  TEST_EQUAL("New set size", test_set->new_tracks(-2)->size(), 1);

  TEST_EQUAL("Percentage tracked", test_set->percentage_tracked(-1,-2), 0.5);
}
