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
