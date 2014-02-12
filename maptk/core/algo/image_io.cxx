/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image_io.h"

#include <maptk/core/algo/algorithm.txx>
#include <maptk/core/exceptions/io.h>
#include <maptk/core/types.h>

#include <boost/filesystem.hpp>

INSTANTIATE_ALGORITHM_DEF(maptk::algo::image_io);


namespace maptk
{

namespace algo
{


image_container_sptr
image_io
::load(std::string const& filename) const
{
  namespace bfs = boost::filesystem;
  if (!bfs::exists(path_t(filename)))
  {
    throw path_not_exists(path_t(filename));
  }
  return this->load_(filename);
}

void
image_io
::save(std::string const& filename, image_container_sptr data) const
{
  this->save_(filename, data);
}


} // end algo namespace

} // end maptk namespace
