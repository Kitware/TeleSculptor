// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Image homography estimation utility
 */

#include <fstream>
#include <string>
#include <vector>

#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>
#include <vital/logger/logger.h>

#include <vital/types/image_container.h>
#include <vital/exceptions.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/vital_types.h>

#include <vital/algo/image_io.h>
#include <vital/algo/convert_image.h>
#include <vital/algo/detect_features.h>
#include <vital/algo/estimate_homography.h>
#include <vital/algo/extract_descriptors.h>
#include <vital/algo/match_features.h>
#include <vital/util/get_paths.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>

#include <maptk/version.h>

typedef kwiversys::SystemTools     ST;
typedef kwiversys::CommandLineArguments argT;

static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( "estimate_homography" ) );

static void print_usage(std::string const &prog_name,
                        argT& args)
{
  std::cout << std::endl
            << "USAGE: " << prog_name << " [OPTS] img1 img2 output_file\n"
            << std::endl
            << "Options:"
            << args.GetHelp() << std::endl
            << "Positional arguments:\n"
            << "    img1 img2   - two in-frame-order image files.\n\n"
            << "    output_file - File to receive generated homography transformation between input frames.\n"
            << "                  This ends up including two homographies: An identity associated to\n"
            << "                  the first frame and then an actual homography describing the\n"
            << "                  transformation to the second frame."
            << std::endl;
}

// Shortcut macro for arbitrarily acting over the tool's algorithm elements.
// ``call`` macro must be two take two arguments: (algo_type, algo_name)
#define tool_algos(call)                                \
  call(detect_features,     feature_detector);          \
  call(extract_descriptors, descriptor_extractor);      \
  call(match_features,      feature_matcher);           \
  call(estimate_homography, homog_estimator);           \
  call(image_io,            image_reader);              \
  call(convert_image,       image_converter)

static kwiver::vital::config_block_sptr default_config()
{
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("homography_estimation_tool");

  // Default algorithm types
  config->set_value("image_reader:type", "vxl");

  config->set_value("image_converter:type", "bypass");

  config->set_value("feature_detector:type", "ocv_SURF");
  config->set_value("feature_detector:ocv_SURF:hessian_threshold", 250);

  config->set_value("descriptor_extractor:type", "ocv_SURF");
  config->set_value("descriptor_extractor:ocv_SURF:hessian_threshold", 250);

  config->set_value("feature_matcher:type", "ocv_flann_based");

  config->set_value("homog_estimator:type", "vxl");

  // expand algo config from defaults above if any
#define get_default(type, name) \
  kwiver::vital::algo::type::get_nested_algo_configuration( #name, config, kwiver::vital::algo::type##_sptr() );

  tool_algos(get_default);

#undef get_default

  return config;
}

static bool check_config(kwiver::vital::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg)                          \
  LOG_WARN(main_logger, "Config Check Fail: " << msg);  \
  config_valid = false

#define check_algo_config(type, name)                             \
  if (! kwiver::vital::algo::type::check_nested_algo_configuration( #name, config )) \
  {                                                                     \
    MAPTK_CONFIG_FAIL("Configuration for algorithm " << #name << " was invalid."); \
  }

  tool_algos(check_algo_config);

#undef check_algo_config

#undef MAPTK_CONFIG_FAIL

  return config_valid;
}

static int maptk_main(int argc, char const* argv[])
{
  //
  // define/parse CLI options
  //
  static bool opt_help(false);
  static std::string opt_config;
  static std::string opt_out_config;
  static double opt_inlier_scale(2.0);
  static std::string opt_mask_image;
  static std::string opt_mask2_image;

  kwiversys::CommandLineArguments arg;
  arg.StoreUnusedArguments(true);

  arg.Initialize( argc, argv );

  arg.AddArgument( "--help",        argT::NO_ARGUMENT, &opt_help, "Display usage information" );
  arg.AddArgument( "-h",            argT::NO_ARGUMENT, &opt_help, "Display usage information" );

  arg.AddArgument( "--config",      argT::SPACE_ARGUMENT, &opt_config,
                   "Optional custom configuration file for the tool. Defaults are set such "
                   "that this is not required.");
  arg.AddArgument( "-c",            argT::SPACE_ARGUMENT, &opt_config,
                   "Optional custom configuration file for the tool. Defaults are set such "
                   "that this is not required.");

  arg.AddArgument( "--output-config", argT::SPACE_ARGUMENT, &opt_out_config,
                   "Output a configuration file with default values. This may be seeded "
                   "with a configuration file from -c/--config.");
  arg.AddArgument( "-o",              argT::SPACE_ARGUMENT, &opt_out_config,
                   "Output a configuration file with default values. This may be seeded "
                   "with a configuration file from -c/--config.");

  arg.AddArgument( "--inlier-scale",  argT::SPACE_ARGUMENT, &opt_inlier_scale,
                   "Error distance tolerated for matches to be considered inliers during "
                   "homography estimation.");
  arg.AddArgument( "-i",              argT::SPACE_ARGUMENT, &opt_inlier_scale,
                   "Error distance tolerated for matches to be considered inliers during "
                   "homography estimation.");

  arg.AddArgument( "--mask_image",   argT::SPACE_ARGUMENT, &opt_mask_image,
                   "Optional boolean mask image where positive values indicate where "
                   "features should be detected. This image *must* be the same size as the "
                   "input images.");
  arg.AddArgument( "-m",            argT::SPACE_ARGUMENT, &opt_mask_image,
                   "Optional boolean mask image where positive values indicate where "
                   "features should be detected. This image *must* be the same size as the "
                   "input images.");

  arg.AddArgument( "--mask-image2", argT::SPACE_ARGUMENT, &opt_mask2_image,
                   "Optional boolean mask image for the second input image. This mask image "
                   "should be provided in the same format as described previously. "
                   "Providing this mask causes the \"--mask-image\" mask to only apply to "
                   "the first image. This mask is only considered if \"--mask-image\" is "
                   "provided.");
  arg.AddArgument( "-n",            argT::SPACE_ARGUMENT, &opt_mask2_image,
                   "Optional boolean mask image for the second input image. This mask image "
                   "should be provided in the same format as described previously. "
                   "Providing this mask causes the \"--mask-image\" mask to only apply to "
                   "the first image. This mask is only considered if \"--mask-image\" is "
                   "provided.");

  if ( ! arg.Parse() )
  {
    std::cerr << "Problem parsing arguments" << std::endl;
    exit( 0 );
  }

  // Process help before anything else
  if( opt_help )
  {
    print_usage( argv[0], arg );
    return EXIT_SUCCESS;
  }

  // only handle positional arguments if the algorithms are to be run
  // if only writing out a config, we don't need the image files
  std::vector<std::string> input_img_files;
  std::string homog_output_path;
  if ( opt_out_config.empty() )
  {
    // Get positional file arguments
    int pos_argc;
    char** pos_argv;

    arg.GetUnusedArguments( &pos_argc, &pos_argv );

    if ( 4 != pos_argc )
    {
      std::cout << "Insufficient number of files specified after options.\n\n";
      print_usage( argv[0], arg );
      return EXIT_FAILURE;
    }

    // Note: pos_argv[0] is the executable name
    input_img_files.push_back( pos_argv[1] );
    input_img_files.push_back( pos_argv[2] );

    homog_output_path = pos_argv[3];
  }

  // register the algorithm implementations
  auto& vpm = kwiver::vital::plugin_manager::instance();
  std::string rel_plugin_path = kwiver::vital::get_executable_path() + "/../lib/kwiver/plugins";
  vpm.add_search_path(rel_plugin_path);
  vpm.load_all_plugins(kwiver::vital::plugin_manager::plugin_type::ALGORITHMS);

  // Set config to algo chain
  // Get config from algo chain after set
  // Check config validity, store result
  //
  // If -o/--output-config given, output config result and notify of current (in)validity
  // Else error if provided config not valid.

  //
  // Setup algorithms and configuration
  //

  namespace algo = kwiver::vital::algo;

  kwiver::vital::config_block_sptr config = default_config();

  // Define algorithm variables.
#define define_algo(type, name)  kwiver::vital::algo::type##_sptr name

  tool_algos(define_algo);

#undef define_algo

  // If -c/--config given, read in confg file, merge onto default just generated
  if( ! opt_config.empty() )
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "telesculptor",
                                                         TELESCULPTOR_VERSION, prefix));
  }

  // Set current configuration to algorithms and extract refined configuration.
#define sa(type, name)                                                       \
  kwiver::vital::algo::type::set_nested_algo_configuration( #name, config, name ); \
  kwiver::vital::algo::type::get_nested_algo_configuration( #name, config, name )

  tool_algos(sa);

#undef sa

  // Check that current configuration is valid.
  bool valid_config = check_config(config);

  if ( ! opt_out_config.empty() )
  {
    write_config_file(config, opt_out_config );
    if(valid_config)
    {
      LOG_INFO(main_logger, "Configuration file contained valid parameters and may be used for running");
    }
    else
    {
      LOG_WARN(main_logger, "Configuration deemed not valid.");
    }
    return EXIT_SUCCESS;
  }
  else if(!valid_config)
  {
    LOG_ERROR(main_logger, "Configuration not valid.");
    return EXIT_FAILURE;
  }

  LOG_INFO(main_logger, "Loading images...");

  kwiver::vital::image_container_sptr i1_image, i2_image;
  try
  {
    i1_image = image_converter->convert(image_reader->load(input_img_files[0]));
    i2_image = image_converter->convert(image_reader->load(input_img_files[1]));
  }
  catch (kwiver::vital::path_not_exists const &e)
  {
    LOG_ERROR(main_logger, e.what());
    return EXIT_FAILURE;
  }
  catch (kwiver::vital::path_not_a_file const &e)
  {
    LOG_ERROR(main_logger, e.what());
    return EXIT_FAILURE;
  }

  // load and convert mask images if they were given
  LOG_DEBUG(main_logger, "Before mask load");
  kwiver::vital::image_container_sptr mask, mask2;
  if( ! opt_mask_image.empty() )
  {
    mask = image_converter->convert( image_reader->load( opt_mask_image ) );

    if( ! opt_mask2_image.empty() )
    {
      mask2 = image_converter->convert( image_reader->load( opt_mask2_image ) );
    }
    else
    {
      mask2 = mask;
    }
  }

  // Make sure we can open for writting the given homography file path
  std::ofstream homog_output_stream( homog_output_path.c_str() );
  if (!homog_output_stream)
  {
    LOG_ERROR(main_logger, "Could not open output homog file: " << homog_output_path );
    return EXIT_FAILURE;
  }

  LOG_INFO(main_logger, "Generating features over input frames...");
  // if no masks were loaded, the value of each mask at this point will be the
  // same as the default value (uninitialized sptr)
  kwiver::vital::feature_set_sptr i1_features = feature_detector->detect(i1_image, mask),
                          i2_features = feature_detector->detect(i2_image, mask2);
  LOG_INFO(main_logger, "Generating descriptors over input frames...");
  kwiver::vital::descriptor_set_sptr i1_descriptors = descriptor_extractor->extract(i1_image, i1_features),
                             i2_descriptors = descriptor_extractor->extract(i2_image, i2_features);
  LOG_INFO(main_logger, "-- Img1 features / descriptors: " << i1_descriptors->size());
  LOG_INFO(main_logger, "-- Img2 features / descriptors: " << i2_descriptors->size());

  LOG_INFO(main_logger, "Matching features...");
  // matching from frame 2 to 1 explicitly. see below.
  kwiver::vital::match_set_sptr matches = feature_matcher->match(i2_features, i2_descriptors,
                                                         i1_features, i1_descriptors);
  LOG_INFO(main_logger, "-- Number of matches: " << matches->size());

  // Because we computed matches from frames 2 to 1, this homography describes
  // the transformation from image2 space to image1 space, which is what
  // warping tools usually want.
  LOG_INFO(main_logger, "Estimating homography...");
  std::vector<bool> inliers;
  kwiver::vital::homography_sptr homog = homog_estimator->estimate(i2_features, i1_features,
                                                           matches, inliers, opt_inlier_scale);
  if( ! homog )
  {
    LOG_ERROR( main_logger, "Failed to estimate valid homography! NULL returned." );
    return EXIT_FAILURE;
  }

  // Reporting inlier count
  size_t inlier_count = 0;
  for(bool b : inliers)
  {
    if (b)
    {
      ++inlier_count;
    }
  }
  LOG_INFO(main_logger, "-- Inliers: " << inlier_count << " / " << inliers.size());

  LOG_INFO(main_logger, "Writing homography file...");
  homog_output_stream << kwiver::vital::homography_<double>() << std::endl
                      << *homog << std::endl;
  homog_output_stream.close();
  LOG_INFO(main_logger, "-- '" << homog_output_path << "' finished writing");

  return EXIT_SUCCESS;
}

int main(int argc, char const* argv[])
{
  try
  {
    return maptk_main(argc, argv);
  }
  catch (std::exception const& e)
  {
    LOG_ERROR(main_logger, "Exception caught: " << e.what());

    return EXIT_FAILURE;
  }
  catch (...)
  {
    LOG_ERROR(main_logger, "Unknown exception caught");

    return EXIT_FAILURE;
  }
}
