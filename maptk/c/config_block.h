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


#ifdef __cplusplus
extern "C"
{
#endif

#include <maptk/c/config.h>

#include <stdbool.h>


/// Structure for opaque pointers to \p config_block objects
typedef struct maptk_config_block_s maptk_config_block_t;


// Config block constants
/// Separator between blocks within the config
MAPTK_C_EXPORT
extern char const *maptk_config_block_block_sep;
/// The magic group for global parameters
MAPTK_C_EXPORT
extern char const *maptk_config_block_global_value;


/// Create a new, empty \p config_block object
/**
 * \return Opaque poitner to an empty config_block with the default name, or 0
 *         if construction failed.
 */
MAPTK_C_EXPORT
maptk_config_block_t* maptk_config_block_new();

/// Create a new, empty \p config_block object with a name
/**
 * \param name String name for the construted config block.
 * \return Opaque poitner to an empty config_block with the default name, or 0
 *         if construction failed.
 */
MAPTK_C_EXPORT
maptk_config_block_t* maptk_config_block_new_named( char const *name );

/// Destroy a config block object
/**
 * If the provided config_block pointer was invalid, this returns 0 signifying
 * nothing was actually destroied.
 *
 * \param cb Opaque pointer to config_block instance.
 * \return 1 if the given config_block was destroyed, else 0 if it was not.
 */
MAPTK_C_EXPORT
unsigned int maptk_config_block_destroy( maptk_config_block_t *cb );

/// Get the name of the \p config_block instance
/**
 * \param cb Opaque pointer to config_block instance.
 * \return String name of the given config_block.
 */
MAPTK_C_EXPORT
char const* maptk_config_block_get_name( maptk_config_block_t *cb );

/// Get a subblock from the configuration.
/**
 * Retrieve an unlinked configuration subblock from the current
 * configuration. Changes made to it do not affect \p *cb.
 *
 * \param cb Opaque pointer to the config_block instance
 * \param key The name of the sub-configuration to retrieve.
 * \return Pointer to a new config_block instance with copies of values.
 */
MAPTK_C_EXPORT
maptk_config_block_t* maptk_config_block_subblock( maptk_config_block_t *cb,
                                                   char const *key );

/// Get a subblock view into the configuration.
/**
 * Retrieve a view into the current configuration. Changes made to \c *cb
 * are seen through the view and vice versa.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param key The name of the sub-configuration to retrieve.
 * \return A subblock which linkes to \p *cb.
 */
MAPTK_C_EXPORT
maptk_config_block_t* maptk_config_block_subblock_view( maptk_config_block_t *cb,
                                                        char const *key );

/// Get the string value for a key
/**
 * This may fail if the key given doesn't exist, returning a null char*.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param key The index of the configuration value to retrieve.
 * \return The string value stored within the configuration.
 */
MAPTK_C_EXPORT
char const* maptk_config_block_get_value( maptk_config_block_t *cb,
                                          char const *key );

/// Get the string value for a key if it exists, else the default
/**
 * \param cb Opaque pointer to a config_block instance.
 * \param key The index of the configuration value to retrieve.
 * \param deflt A default value to return if the given key does not have an
 *              associated value.
 * \return the \p char* value stored within the configuration.
 */
MAPTK_C_EXPORT
char const* maptk_config_block_get_value_default( maptk_config_block_t *cb,
                                                  char const *key,
                                                  char const *deflt );

/// Get the description associated to a value
/**
 * If the provided key exists but has no description associated with it, an
 * empty string is returned.
 *
 * If the key provided does not exist, a NULL pointer is returned.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param key The name of the parameter to get the description of.
 * \returns The string description of the give key or NULL if the key was not
 *          found.
 */
MAPTK_C_EXPORT
char const* maptk_config_block_get_description( maptk_config_block_t *cb,
                                                char const *key );

/// Set a string value within the configuration.
/**
 * If the provided key has been marked read-only, nothing is set.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param key The index of the configuration value to set.
 * \param value The value to set for the \p key.
 */
MAPTK_C_EXPORT
void maptk_config_block_set_value( maptk_config_block_t *cb,
                                   char const *key,
                                   char const *value );

/// Set a string value with an associated description
/**
 * If the provided key is marked read-only, nothing is set.
 *
 * If this key already exists, has a description and no new description
 * was passed with this \c set_value call, the previous description is
 * retained. We assume that the previous description is still valid and
 * this a value overwrite. If it is intended for the description to also
 * be overwritted, an \c unset_value call should be performed on the key
 * first, and then this \c set_value call.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param key The index of the configuration value to set.
 * \param value The value to set for the \p key.
 * \param descr Description of the key. If this is set, we will override
 *              any existing description for the given key. If a
 *              description for the given key already exists and nothing
 *              was provided for this parameter, the existing description
 *              is maintained.
 */
MAPTK_C_EXPORT
void maptk_config_block_set_value_descr( maptk_config_block_t *cb,
                                         char const *key,
                                         char const *value,
                                         char const *description );

/// Remove a key/value pair from the configuration.
/**
 * If the provided key is marked as read-only, nothing is unset.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param key The index of the configuration value to set.
 */
MAPTK_C_EXPORT
void maptk_config_block_unset_value( maptk_config_block_t *cb,
                                     char const *key );

/// Query if a value is read-only
/**
 * \param cb Opaque pointer to a config_block instance.
 * \param key The key to check.
 * \returns True if the \p key has been marked as read-only, and false
 *          otherwise.
 */
MAPTK_C_EXPORT
bool maptk_config_block_is_read_only( maptk_config_block_t *cb,
                                      char const *key );

/// Mark the given key as read-only
/**
 * This provided key is marked as read-only even if it doesn't currently
 * exist in the given config_block instance.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param key The key to mark as read-only.
 */
MAPTK_C_EXPORT
void maptk_config_block_mark_read_only( maptk_config_block_t *cb,
                                        char const *key );

/// Merge the values in \p other into the current config \p cb.
/**
 * A partial merger will occur if the merger would attempt overwriting a key
 * marked as read-only.
 *
 * \note Any values currently set within \c *this will be overwritten if
 *       conficts occur.
 *
 * \param cb Opaque pointer to a config_block instance.
 * \param other Opaque pointer to a config_block instance whose key/value
 *              pairs are to be merged into \p cb.
 */
MAPTK_C_EXPORT
void maptk_config_block_merge_config( maptk_config_block_t *cb,
                                      maptk_config_block_t *other );

/// Check if a value exists for the given key
/**
 * \param cb Opaque pointer to a config_block instance.
 * \param key The index of the configuration value to check.
 * \return 1 if \p cb has a value for the given \p key, else 0.
 */
MAPTK_C_EXPORT
bool maptk_config_block_has_value( maptk_config_block_t *cb,
                                   char const *key );

/// Return the values available in the configuration.
/**
 * We are expecting that the \p length and \p keys parameters will be passed
 * by reference by the user as they are dereferenced within the function for
 * value assignment.
 *
 * \param[in] cb Opaque pointer to a config_block instance.
 * \param[out] length The number of available keys in \p cb.
 * \param[out] keys Pointer to an array of char* strings.
 */
MAPTK_C_EXPORT
void maptk_config_block_available_values( maptk_config_block_t *cb,
                                          unsigned int *length,
                                          char ***keys );


#ifdef __cplusplus
}
#endif


#endif // MAPTK_C_CONFIG_BLOCK_H_
