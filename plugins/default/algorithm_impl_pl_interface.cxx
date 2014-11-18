
#include <maptk/plugin_interface/algorithm_impl_pl_interface.h>

#include "close_loops_bad_frames_only.h"
#include "close_loops_multi_method.h"
#include "compute_ref_homography_default.h"
#include "hierarchical_bundle_adjust.h"
#include "match_features_homography.h"
#include "plugin_default_config.h"
#include "track_features_default.h"


extern "C"
PLUGIN_DEFAULT_EXPORT
int register_algo_impls(void)
{
  int registered =
    maptk::algo::close_loops_bad_frames_only::register_self() +
    maptk::algo::close_loops_multi_method::register_self() +
    maptk::algo::compute_ref_homography_default::register_self() +
    maptk::algo::hierarchical_bundle_adjust::register_self() +
    maptk::algo::match_features_homography::register_self() +
    maptk::algo::track_features_default::register_self();
  return 6 - registered;
}
