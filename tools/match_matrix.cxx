// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief compute a match matrix from a track file
 */

#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <vector>

#include <unsupported/Eigen/SparseExtra>

#include <arrows/core/match_matrix.h>
#include <vital/exceptions.h>
#include <vital/io/track_set_io.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>

typedef kwiversys::CommandLineArguments argT;
typedef kwiversys::SystemTools     ST;

/// Report usage message to std::cerr
void usage( char const* argv[], argT& arg )
{
    std::cerr << "Usage: " << argv[0] << " [OPTS]\n"
              << "\n"
              << "Read a track file and compute the match matrix\n"
              << "\n"
              << "Options:\n"
              << arg.GetHelp()
              << std::endl;
}

// ------------------------------------------------------------------
void
write_frame_numbers(std::ostream& os,
                    const std::vector<kwiver::vital::frame_id_t>& frames)
{
  for( unsigned i=0; i<frames.size(); ++i )
  {
    os << frames[i] << " ";
  }
  os << std::endl;
}

// ------------------------------------------------------------------
void
write_match_matrix(std::ostream& os,
                   const Eigen::SparseMatrix<unsigned int>& mm)
{
  // TODO write out the matrix in a more memory efficient way
  os << Eigen::MatrixXd(mm.cast<double>()) << std::endl;
}

// ------------------------------------------------------------------
void
check_file_path(const kwiver::vital::path_t& fp)
{
  using namespace kwiver;
  // If the given path is a directory, we obviously can't write to it.
  if( ST::FileIsDirectory(fp))
  {
    throw vital::file_write_exception(fp, "Path given is a directory, "
                                          "can not write file.");
  }

  // Check that the directory of the given file path exists,
  // creating necessary directories where needed.
  vital::path_t parent_dir = ST::GetFilenamePath(
    ST::CollapseFullPath( fp ) );
  if( ! ST::FileIsDirectory(parent_dir))
  {
    if( ! ST::MakeDirectory(parent_dir))
    {
      throw vital::file_write_exception(parent_dir, "Attempted directory creation, "
                                                    "but no directory created! No "
                                                    "idea what happened here...");
    }
  }
  std::ofstream ofs(fp.c_str());
}

// ------------------------------------------------------------------
static int maptk_main(int argc, char const* argv[])
{
  using namespace kwiver;

  static bool        opt_help( false );
  static std::string opt_in_tracks;
  static std::string opt_out_matrix;
  static std::string opt_out_frames;

  kwiversys::CommandLineArguments arg;

  arg.Initialize( argc, argv );

  arg.AddArgument( "--help",           argT::NO_ARGUMENT, &opt_help, "Display usage information" );
  arg.AddArgument( "-h",               argT::NO_ARGUMENT, &opt_help, "Display usage information" );
  arg.AddArgument( "--input-tracks",   argT::SPACE_ARGUMENT, &opt_in_tracks, "Input track file." );
  arg.AddArgument( "--output-matrix",  argT::SPACE_ARGUMENT, &opt_out_matrix, "Output match matrix file" );
  arg.AddArgument( "--output-frames",  argT::SPACE_ARGUMENT, &opt_out_frames, "Output frame number file" );

  if ( ! arg.Parse() )
  {
    std::cerr << "Problem parsing arguments" << std::endl;
    return EXIT_FAILURE;
  }

  if ( opt_help )
  {
    usage( argv, arg );
    return EXIT_SUCCESS;
  }

  if( opt_in_tracks.empty() )
  {
    usage( argv, arg );
    return EXIT_FAILURE;
  }

  // test the output files
  if( ! opt_out_matrix.empty() )
  {
    vital::path_t outfile( opt_out_matrix );
    check_file_path(outfile);
  }

  if( ! opt_out_frames.empty() )
  {
    vital::path_t outfile( opt_out_frames );
    check_file_path(outfile);
  }

  // load the tracks
  std::string infile = opt_in_tracks;
  std::cout << "loading: "<< infile << std::endl;
  vital::track_set_sptr tracks = vital::read_track_file(infile);

  // compute the match matrix
  std::cout << "computing matching matrix" <<std::endl;
  std::vector<vital::frame_id_t> frames;
  Eigen::SparseMatrix<unsigned int> mm = kwiver::arrows::match_matrix(tracks, frames);

  // write output
  if( ! opt_out_matrix.empty() )
  {
    vital::path_t outfile( opt_out_matrix );
    std::cout << "writing matrix to: "<< outfile << std::endl;
    if( ST::GetFilenameExtension( outfile ) == ".mtx" )
    {
      Eigen::saveMarket(mm, outfile);
    }
    else
    {
      std::ofstream ofs(outfile.c_str());
      write_match_matrix(ofs, mm);
    }
  }
  else
  {
    write_frame_numbers(std::cout, frames);
    write_match_matrix(std::cout, mm);
  }

  if( ! opt_out_frames.empty() )
  {
    vital::path_t outfile( opt_out_frames );
    std::cout << "writing frame numbers to: "<< outfile << std::endl;
    std::ofstream ofs(outfile.c_str());
    write_frame_numbers(ofs, frames);
  }

  return EXIT_SUCCESS;
}

// ------------------------------------------------------------------
int main(int argc, char const* argv[])
{
  try
  {
    return maptk_main(argc, argv);
  }
  catch (std::exception const& e)
  {
    std::cerr << "Exception caught: " << e.what() << std::endl;

    return EXIT_FAILURE;
  }
  catch (...)
  {
    std::cerr << "Unknown exception caught" << std::endl;

    return EXIT_FAILURE;
  }
}
