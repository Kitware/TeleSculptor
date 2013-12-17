/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CORE_TYPES_H
#define MAPTK_CORE_TYPES_H

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>

/**
 * \file
 * \brief Shared type declarations for the \ref maptk::core module.
 */

namespace maptk
{

class config;
typedef boost::shared_ptr<config> config_t;

/// The type that represents a configuration value key.
typedef std::string config_key_t;
/// The type that represents a collection of configuration keys.
typedef std::vector<config_key_t> config_keys_t;
/// The type that represents a stored configuration value.
typedef std::string config_value_t;
/// The type that represents a description of a configuration key.
typedef std::string config_description_t;

/// The type to be used for file and directory paths
typedef boost::filesystem::path path_t;

/// The type for a static token in the config parser
typedef std::string token_t;

}

#endif // MAPTK_CORE_TYPES_H
