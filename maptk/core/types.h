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
 * \brief Shared type declarations for the maptk/core module.
 */

#ifndef MAPTK_CORE_TYPES_H
#define MAPTK_CORE_TYPES_H

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>


namespace maptk
{

/// The type that represents a configuration value key.
typedef std::string config_block_key_t;
/// The type that represents a collection of configuration keys.
typedef std::vector<config_block_key_t> config_block_keys_t;
/// The type that represents a stored configuration value.
typedef std::string config_block_value_t;
/// The type that represents a description of a configuration key.
typedef std::string config_block_description_t;

/// The type to be used for file and directory paths
typedef boost::filesystem::path path_t;

/// The type for a static token in the config_block parser
typedef std::string token_t;

/// The type of a landmark ID number
typedef unsigned int landmark_id_t;

/// The type of a track ID number
typedef unsigned int track_id_t;

/// The type of a frame number or camera ID
typedef unsigned int frame_id_t;

}

#endif // MAPTK_CORE_TYPES_H
