/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of load/save wrapping functionality.
 */

#include "image_io.h"

#include <maptk/core/algo/algorithm.txx>
#include <maptk/core/exceptions/io.h>
#include <maptk/core/types.h>

#include <boost/filesystem.hpp>


/// \cond DoxygenSuppress
INSTANTIATE_ALGORITHM_DEF(maptk::algo::image_io);
/// \endcond


namespace maptk
{

namespace algo
{


image_container_sptr
image_io
::load(std::string const& filename) const
{
  // Make sure that the given file path exists and is a file.
  namespace bfs = boost::filesystem;
  if (!bfs::exists(filename))
  {
    throw path_not_exists(filename);
  }
  else if (!bfs::is_regular_file(filename))
  {
    throw path_not_a_file(filename);
  }

  return this->load_(filename);
}

void
image_io
::save(std::string const& filename, image_container_sptr data) const
{
  // Make sure that the given file path's containing directory exists and is
  // actually a directory.
  namespace bfs = boost::filesystem;
  path_t containing_dir = bfs::absolute(path_t(filename)).parent_path();
  if (!bfs::exists(containing_dir))
  {
    throw path_not_exists(containing_dir);
  }
  else if (!bfs::is_directory(containing_dir))
  {
    throw path_not_a_directory(containing_dir);
  }

  this->save_(filename, data);
}


} // end algo namespace

} // end maptk namespace
