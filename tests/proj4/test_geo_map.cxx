/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <cstdio>
#include <maptk/proj4/geo_map.h>

// test_common.h required things
#define TEST_ARGS ()
DECLARE_TEST_MAP();

// distance epsilons for centimeter accuracy or better
static const double met_epsilon = 0.01;
static const double deg_epsilon = 0.0000001;

int main(int argc, char** argv)
{
  CHECK_ARGS(1);
  testname_t const testname = argv[1];
  RUN_TEST(testname);
}

bool is_about(double value, double target, double epsilon)
{
  return (value <= (target + epsilon)) && (value >= (target - epsilon));
}

IMPLEMENT_TEST(latlon_to_utm)
{
  maptk::algo::proj_geo_map gm;
  double lat = -17.234908,
         lon = 24.000048,
         easting, northing;
  int zone;
  bool is_northp;

  gm.latlon_to_utm(lat, lon, easting, northing, zone, is_northp);
  printf("ret UTM: %.2fE %.2fN %d %d\n", easting, northing, zone, is_northp);

  double expected_easting = 180954.88,
         expected_northing = -1908018.40;
  int expected_zone = 35;
  bool expected_np = true;
  if(! is_about(easting, expected_easting, met_epsilon))
    TEST_ERROR("Result UTM easting more than a centimeter off expected "
               << "value of " << expected_easting << ". "
               << "(got " << easting << ")");
  if(! is_about(northing, expected_northing, met_epsilon))
    TEST_ERROR("Result UTM northing more than a centimeter off expected "
               << "value of " << expected_easting << ". "
               << "(got " << easting << ")");
  if(! zone == expected_zone)
    TEST_ERROR("Result UTM zone not equal to expected (" << expected_zone
               << " expected, got " << zone << ")");
  if(! is_northp == expected_np)
    TEST_ERROR("Result point does not match expected \"is north point\" "
               << "result. (expected " << expected_np
               << ", got " << is_northp << ")");

}

IMPLEMENT_TEST(utm_to_latlon)
{
  maptk::algo::proj_geo_map gm;
  double easting = 180954.88,
         northing = -1908018.40,
         lat, lon,
         ex_lat = -17.234908,
         ex_lon = 24.000048;
  int zone = 35;
  bool is_np = true;

  gm.utm_to_latlon(easting, northing, zone, is_np, lat, lon);

  if(! is_about(lat, ex_lat, deg_epsilon))
    printf("Error: lat check failed.\n"
           "\tExpected %.16f\n"
           "\tGot      %.16f\n",
           ex_lat, lat);

  if(! is_about(lon, ex_lon, deg_epsilon))
    printf("Error: lon check failed.\n"
           "\tExpected %.16f\n"
           "\tGot      %.16f\n",
           ex_lon, lon);
}

IMPLEMENT_TEST(backprojection)
{
  maptk::algo::proj_geo_map gm;
  double orig_lat = -17.234908,
         orig_lon = 24.000048,
         orig_easting = 180954.88,
         orig_northing = -1908018.40;
  int orig_zone = 35;
  bool orig_is_np = true;

  double lat, lon, easting, northing;
  int zone;
  bool is_np;

  // lat/lon back projection
  gm.latlon_to_utm(orig_lat, orig_lon,
                   easting, northing, zone, is_np);
  gm.utm_to_latlon(easting, northing, zone, is_np,
                   lat, lon);

  if(! is_about(lat, orig_lat, deg_epsilon))
    printf("Error: back-projection lat check failed.\n"
           "\tExpected %.16f\n"
           "\tGot      %.16f\n",
           orig_lat, lat);
  if(! is_about(lon, orig_lon, deg_epsilon))
    printf("Error: back-projection lon check failed.\n"
           "\tExpected %.16f\n"
           "\tGot      %.16f\n",
           orig_lon, lon);

  // UTM back projection
  gm.utm_to_latlon(orig_easting, orig_northing, orig_zone, orig_is_np,
                   lat, lon);
  gm.latlon_to_utm(lat, lon,
                   easting, northing, zone, is_np);

  if(! is_about(easting, orig_easting, met_epsilon))
    printf("Error: back-projection easting check failed.\n"
           "\tExpected %.3f\n"
           "\tGot      %.3f\n",
           orig_easting, easting);
  if(! is_about(northing, orig_northing, met_epsilon))
    printf("Error: back-projection northing check failed.\n"
           "\tExpected %.3f\n"
           "\tGot      %.3f\n",
           orig_northing, northing);
  if(! (zone == orig_zone))
    printf("Error: back-projection zone check failed.\n"
           "\tExpected %d\n"
           "\tGot      %d\n",
           orig_zone, zone);
  if(! (is_np == orig_is_np))
    printf("Error: back-projection north-point check failed.\n"
           "\tExpected %d\n"
           "\tGot      %d\n",
           orig_is_np, is_np);
}
