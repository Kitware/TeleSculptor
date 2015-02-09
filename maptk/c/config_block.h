/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief C Interface to \p config_block object definitions
 */

#ifndef MAPTK_C_CONFIG_BLOCK_H_
#define MAPTK_C_CONFIG_BLOCK_H_


#include <maptk/c/config.h>


#ifdef __cplusplus
extern "C"
{
#endif


/// Structure for opaque pointers to \p config_block objects
typedef struct config_block_s config_block_s;

/// Create a new, empty \p config_block object
/**
 * \return Opaque poitner to an empty config_block with the default name, or 0
 *         if construction failed.
 */
MAPTK_C_EXPORT
config_block_s* maptk_config_block_new();

/// Create a new, empty \p config_block object with a name
/**
 * \param name String name for the construted config block.
 * \return Opaque poitner to an empty config_block with the default name, or 0
 *         if construction failed.
 */
MAPTK_C_EXPORT
config_block_s* maptk_config_block_new_named( char const *name );

/// Destroy a config block object
/**
 * If the provided config_block pointer was invalid, this returns 0 signifying
 * nothing was actually destroied.
 *
 * \param cb Opaque pointer to config_block instance.
 * \return 1 if the given config_block was destroyed, else 0 if it was not.
 */
MAPTK_C_EXPORT
unsigned int maptk_config_block_destroy( config_block_s *cb );

/// Get the name of the \p config_block instance
/**
 * \param cb Opaque pointer to config_block instance.
 * \return String name of the given config_block.
 */
MAPTK_C_EXPORT
char const* maptk_config_block_get_name( config_block_s *cb );

/// Get a copy of a sub-block of the configuration
/**
 * \param cb Opaque pointer to the config_block instance
 * \param key String key to get the subblock from.
 * \return Pointer to a new config_block instance
 */
MAPTK_C_EXPORT
config_block_s* maptk_config_block_subblock( config_block_s *cb,
                                             char const *key );

/// Get a mutable view of a sub-block within a configuration
MAPTK_C_EXPORT
config_block_s* maptk_config_block_subblock_view( config_block_s *cb,
                                                  char const *key );

///// Get the string value for a key
//MAPTK_C_EXPORT
//char* maptk_config_block_get_value( config_block_s *cb,
//                                    char* key );

///// Get the value for a key as an int
//MAPTK_C_EXPORT
//int maptk_config_block_get_value_int( config_block_s *cb,
//                                      char* key );
///// Get the value for a key as a float
//MAPTK_C_EXPORT
//float maptk_config_block_get_value_float( config_block_s *cb,
//                                          char* key );
///// Get the value for a key as a double
//MAPTK_C_EXPORT
//double maptk_config_block_get_value_double( config_block_s *cb,
//                                            char* key );

///// Get the string value for a key if it exists, else the default
//MAPTK_C_EXPORT
//char*  maptk_config_block_get_value_default( config_block_s *cb,
//                                             char* key,
//                                             char* default );

///// Get the value for a key as an int if it exists, else the default
//MAPTK_C_EXPORT
//int maptk_config_block_get_value_default_int( config_block_s *cb,
//                                              char* key,
//                                              int default );
///// Get the value for a key as a float if it exists, else the default
//MAPTK_C_EXPORT
//float maptk_config_block_get_value_default_float( config_block_s *cb,
//                                                  char* key,
//                                                  float default );
///// Get the value for a key as a double if it exists, else the default
//MAPTK_C_EXPORT
//double maptk_config_block_get_value_default_double( config_block_s *cb,
//                                                    char* key,
//                                                    double default );

///// Get the description string for a given key
//MAPTK_C_EXPORT
//char* maptk_config_block_get_description( config_block_s *cb,
//                                          char* key );
//
///// Set the string value for a key
//MAPTK_C_EXPORT
//void maptk_config_block_set_value( config_block_s *cb,
//                                   char *key,
//                                   char *value );
//
///// Merge another \p config_block's entries into this \p config_block
//MAPTK_C_EXPORT
//void maptk_config_block_merge_config( config_block_s *cb,
//                                      config_block_s *other );
//
///// Check if a value exists for the given key
//MAPTK_C_EXPORT
//int maptk_config_block_has_key( config_block_s *cb,
//                                char *key );


#ifdef __cplusplus
}
#endif


#endif // MAPTK_C_CONFIG_BLOCK_H_
