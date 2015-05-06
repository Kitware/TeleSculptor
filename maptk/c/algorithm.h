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
 * \brief C interface to base algorithm/_def/_impl classes
 */

#ifndef MAPTK_C_ALGORITHM_H_
#define MAPTK_C_ALGORITHM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include <maptk/c/common.h>
#include <maptk/c/config.h>
#include <maptk/c/config_block.h>
#include <maptk/c/error_handle.h>
#include <maptk/c/image_container.h>


/// Opaque pointer to a MAPTK Algorithm instance
typedef struct maptk_algorithm_s maptk_algorithm_t;


// ===========================================================================
// Static algorithm methods
//
// These methods usually require one or more of the algorithm type and
// implementation labels.
// ---------------------------------------------------------------------------


/// Check the given type label against registered algorithm types
/**
 * Returns false if the there is no registered algorithm implementation with
 * given type label.
 *
 * \param type_name Type label string to check for.
 * \param eh Error handle instance pointer.
 * \return True if there exists a registered algorithm implementation with the
 *         given type label. False otherwise.
 */
MAPTK_C_EXPORT
bool
maptk_algorithm_has_type_name( char const *type_name,
                               maptk_error_handle_t *eh );


/// Check the given type and implementation names against registered algorithms
/**
 * \param type_name Type name of algorithm to validate
 * \param impl_name Implementation name of algorithm to validate
 * \param eh Error handle instance pointer.
 * \return True if the given \p type_name and \p impl_name describe a valid
 *         registered algorithm, or false if not.
 */
MAPTK_C_EXPORT
bool
maptk_algorithm_has_impl_name( char const *type_name,
                               char const *impl_name,
                               maptk_error_handle_t *eh );


/// Return an array of impl names of registered algorithms of the given type
/**
 * \note If the given \p type_name is an empty string, we return all registered
 *       algorithms of any type, returning strings of the form
 *       "<type_name>:<impl_name>". When a \p type_name is specified, returned
 *       strings are simply the available implementation names.
 *
 * \note If there are no resunts for a given query, \p length will be set to 0
 *       and \p *names will be set to NULL.
 *
 * \param type_name Type name of algorithm for which to return available
 *    implementation names, or an empty string.
 * \param length Return pointer which will be set to the number of names for
 *    the given query.
 * \param names Return pointer which will be set to the array of returned
 *    strings, or NULL if there are no return elements.
 * \param eh Error handle instance pointer.
 */
MAPTK_C_EXPORT
void
maptk_algorithm_registered_names( char const *type_name,
                                  unsigned int *length,
                                  char ***names,
                                  maptk_error_handle_t *eh );


/// Create a new algorithm instance of the requested type and implementation
/**
 * The given type and implementation labels must match a registered type/impl
 * pairing.
 *
 * \param type_name Label of the algorithm type
 * \param impl_name Label of the algorithm name
 * \return New algorithm implementation structure with a new implementation
 *         instance.
 */
MAPTK_C_EXPORT
maptk_algorithm_t*
maptk_algorithm_create( char const *type_name,
                        char const *impl_name,
                        maptk_error_handle_t *eh );


/// Helper function for properly getting a nested algorithm's configuration
/**
 * Adds a configurable algorithm implementation parameter to the given
 * \p cb for this algorithm.
 *
 * If the variable pointed to by \p algo is a defined \p maptk_algorithm_t
 * instance, its configuration parameters are merged with the given
 * \p maptk_config_block_t instance.
 *
 * The algorithm's \p type_name is needed so we can properly document
 * generic parameters regardless of the value of \p algo.
 *
 * \param type_name The type name of the nested algorithm.
 * \param name      An identifying name for the nested algorithm.
 * \param[in] cb    The \p maptk_config_block_t instance in which to put the
 *                    generic switch and nested algorithm's configuration.
 * \param[in] algo  The nested algorithm instance, or NULL if there isn't one.
 */
MAPTK_C_EXPORT
void
maptk_algorithm_get_nested_algo_configuration( char const *type_name,
                                               char const *name,
                                               maptk_config_block_t *cb,
                                               maptk_algorithm_t *algo,
                                               maptk_error_handle_t *eh );


/// Helper function for properly setting a nested algorithm's configuration
/**
 * If the value for the config parameter \p type is supported by the concrete
 * algorithm class, then a new algorithm instance is created, configured and
 * returned via the \p algo pointer.
 *
 * The \p algo pointer will not be set/modified if the implementation type
 * (parameter defined in \c maptk_algorithm_get_nested_algo_configuration)
 * is not present or set to an invalid value relative to the registered names
 * for the given \p type_name.
 *
 * The algorithm \p type_name must be provided so that implementation type can
 * be correctly validated as well as for correct production of the algorithm
 * instance.
 *
 * This will always create a new algorithm implementation instance in which
 * to set parameters when the configured implementation type is valid.
 *
 * \param type_name The type name of the nested algorithm
 * \param name      An identifying name for the nested algorithm
 * \param[in] cb    The \p maptk_config_block_t instance from which we will
 *                    draw configuration needed for the nested algorithm
 *                    instance.
 * \param[out] algo The reference to the nested \p maptk_algorithm_t pointer.
 */
MAPTK_C_EXPORT
void
maptk_algorithm_set_nested_algo_configuration( char const *type_name,
                                               char const *name,
                                               maptk_config_block_t *cb,
                                               maptk_algorithm_t **algo,
                                               maptk_error_handle_t *eh );


/// Helper function for checking that basic nested algorithm configuration validity
/**
 * Check that the expected implementation switch exists and that its value is
 * a registered implementation name.
 *
 * If the name is valid, we also recursively check the configuration of
 * the set implementation. This is done with a fresh create so we don't have to
 * rely on the implementation being defined in the instance that is called
 * from.
 *
 * \param type_name The type name of the nested algorithm.
 * \param name      An identifying name for the nested algorithm.
 * \param cb        The \p maptk_config_block_t instance to check.
 * \return  True if the given confing block checks out for the given algorithm
 *          type.
 */
MAPTK_C_EXPORT
bool
maptk_algorithm_check_nested_algo_configuration( char const *type_name,
                                                 char const *name,
                                                 maptk_config_block_t *cb,
                                                 maptk_error_handle_t *eh );


// ===========================================================================
// Functions on general algorithm pointer
// ---------------------------------------------------------------------------

/// Return the name of this algorithm
/**
 * \param algo Opaque pointer to algorithm instance.
 * \param eh Error handle instance pointer
 * \return String name of the algorithm type.
 */
MAPTK_C_EXPORT
maptk_string_t*
maptk_algorithm_type_name( maptk_algorithm_t *algo,
                           maptk_error_handle_t *eh );


/// Return the name of this implementation
/**
 * \param algo Opaque pointer to algorithm instance.
 * \param eh Error handle instance pointer
 * \return String name of the algorithm implementation type.
 */
MAPTK_C_EXPORT
maptk_string_t*
maptk_algorithm_impl_name( maptk_algorithm_t *algo,
                           maptk_error_handle_t *eh );


/// Return optional descriptive string about an implementation
/**
 * \param algo Opaque pointer to an algorithm instance.
 * \param eh Error handle instance pointer
 * \return String description for the given algorithm, or an empty string
 *         if there is no available description.
 */
MAPTK_C_EXPORT
maptk_string_t*
maptk_algorithm_description( maptk_algorithm_t *algo,
                             maptk_error_handle_t *eh );


/// Get an algorithm implementation's configuration block
MAPTK_C_EXPORT
maptk_config_block_t*
maptk_algorithm_get_impl_configuration( maptk_algorithm_t *algo,
                                        maptk_error_handle_t *eh );


/// Set this algorithm implementation's properties via a config block
MAPTK_C_EXPORT
void
maptk_algorithm_set_impl_configuration( maptk_algorithm_t *algo,
                                        maptk_config_block_t *cb,
                                        maptk_error_handle_t *eh );


/// Check that the algorithm implementation's configuration is valid
MAPTK_C_EXPORT
bool
maptk_algorithm_check_impl_configuration( maptk_algorithm_t *algo,
                                          maptk_config_block_t *cb,
                                          maptk_error_handle_t *eh );


/// Clone the given algorithm instance
/**
 * If a NULL pointer is given, a NULL pointer is returned, as there is nothing
 * to clone.
 *
 * \param algo The algorithm instance to clone.
 * \param eh Error handle instance pointer.
 * \return The new, cloned algorithm instance pointer.
 */
MAPTK_C_EXPORT
maptk_algorithm_t*
maptk_algorithm_clone( maptk_algorithm_t *algo,
                       maptk_error_handle_t *eh );


/// Destroy the given algorithm instance
/**
 * \param algo Algorithm instance pointer to destroy.
 * \param eh Error handle instance pointer.
 */
MAPTK_C_EXPORT
void
maptk_algorithm_destroy( maptk_algorithm_t *algo,
                         maptk_error_handle_t *eh );


#ifdef __cplusplus
}
#endif

#endif //MAPTK_C_ALGORITHM_H_
