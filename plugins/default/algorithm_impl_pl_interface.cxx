
#include <maptk/plugin_interface/algorithm_impl_pl_interface.h>

#include "close_loops_bad_frames_only.h"
#include "close_loops_multi_method.h"
#include "compute_ref_homography_default.h"
#include "convert_image_default.h"
#include "hierarchical_bundle_adjust.h"
#include "match_features_homography.h"
#include "plugin_default_config.h"
#include "track_features_default.h"

#include <iostream>

#include <maptk/logging_macros.h>


#ifdef __cplusplus
extern "C"
{
#endif


PLUGIN_DEFAULT_EXPORT
int register_algo_impls(maptk::registrar &reg)
{
  using namespace std;
  cerr << "Registering DEFAULT plugin algo implementations" << endl;

  int registered
    = maptk::algo::close_loops_bad_frames_only::register_self(reg)
    + maptk::algo::close_loops_multi_method::register_self(reg)
    + maptk::algo::compute_ref_homography_default::register_self(reg)
    + maptk::algo::convert_image_default::register_self(reg)
    + maptk::algo::hierarchical_bundle_adjust::register_self(reg)
    + maptk::algo::match_features_homography::register_self(reg)
    + maptk::algo::track_features_default::register_self(reg)
    ;

  cerr << "Registered algorithms. Returned: " << registered << endl;
  return 7 - registered;
}


#ifdef __cplusplus
}
#endif
