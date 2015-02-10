/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief algorithm regstrar class
 */

#ifndef MAPTK_REGISTRAR_H_
#define MAPTK_REGISTRAR_H_

#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <maptk/config.h>


namespace maptk
{


/**
 * \brief A singleton class to keep a shared-pointer registry of objects across
 *        multiple algorithm definition types
 *
 * NOTE: Do not use the static instance method ``instance()`` within plugins!
 * When the main library is built statically, the main library and plugin
 * libraries have unique static varaible spaces, and thus to not share a
 * singleton instance. Only use the registrar passed to the C interface
 * methods.
 */
class MAPTK_LIB_EXPORT registrar
{
protected:
  template <typename T>
  class reg_type
  {
  public:
    typedef std::map< std::string, boost::shared_ptr<T> > map;
    typedef typename map::value_type pair;
    typedef typename map::const_iterator const_itr;
  };

public:

  /// Access the singleton instance of this class
  /**
   * \returns The reference to the singleton instance.
   */
  static registrar& instance()
  {
    static registrar instance_ = registrar();
    return instance_;
  }

  /// Register a new name and item with the registrar for a given type
  /**
   * This will overwrite an existing entry if \p name already exists in the
   * registration map for the given type \p T.
   *
   * A false value is returned if a NULL item pointer is given.
   *
   * \tparam Algorithm definition type of the given instance.
   * \param name The string label to associate to this algorithm instance
   * \param item Algorithm instance descending from definition type \p T to act
   *             as the default instance for the given label.
   * \returns True if the given \p name / \p item pair was successfully
   *          inserted, and false if not.
   */
  template <typename T>
  bool register_item(const std::string& name, boost::shared_ptr<T> item)
  {
    if( ! item )
    {
      return false;
    }

    // For logging
    std::ostringstream ss;
    ss << "Registering algo implementation '" << name << "' for def type '"
       << item->type_name() << "'";
    this->debug_msg( ss.str() );

    // declaring map reference so we don't copy
    typename reg_type<T>::map &im = this->get_item_map<T>();
    im[name] = item;
    return true;
  }

  /// Return a vector of registered item names for a given type
  /**
   * \tparam T Algorithm definition type to look up registered labels for.
   * \returns Vector of strings that are the label names that are currently
   *          registered for the given algorithm definition type \p T.
   */
  template <typename T>
  std::vector<std::string> registered_names()
  {
    std::vector<std::string> names;
    BOOST_FOREACH( typename reg_type<T>::pair i, this->get_item_map<T>() )
    {
      names.push_back(i.first);
    }
    return names;
  }

  /// Check if a given algorithm label is registered for the given type
  /**
   * \param name The algorithm instance label to check for in the type's
   *             registration.
   * \returns True if the given name exists in the given type's registration and
   *          false if not.
   */
  template <typename T>
  bool is_registered( std::string name )
  {
    return this->get_item_map<T>().count( name ) > 0;
  }

  /// Find the item matching \a name or return NULL
  /**
   * \tparam T Algorithm definition type to constrain to.
   * \param[in] name The string label of an algorithm of definition \p T to
   *                 find in this registrar
   * \returns Shared pointer to the instnace associated to the given label.
   *          If no algorithm is found, an empty shared pointer is returned.
   */
  template <typename T>
  boost::shared_ptr<T> find(const std::string& name)
  {
    typename reg_type<T>::map &im = this->get_item_map<T>();
    typename reg_type<T>::const_itr it = im.find(name);
    if (it == im.end())
    {
      return boost::shared_ptr<T>();
    }
    return it->second;
  }

private:

  /// For an algorithm_def type, return the associated static item map.
  /**
   * \tparam T Algorithm definition type to get the mapping for.
   */
  template <typename T>
  typename reg_type<T>::map &
  get_item_map()
  {
    // Static mapping of name-to-instance for a specific algorithm type
    // Due to the use of this class as a strict singleton, static variable
    //  use is acceptable here.
    static typename reg_type<T>::map ad_item_map_ = typename reg_type<T>::map();
    return ad_item_map_;
  }

  /// Private constructor (this class is a singleton)
  registrar() {}
  /// Private destructor (this class is a singleton)
  ~registrar() {}
  /// Private copy constructor (this class is a singleton)
  registrar(const registrar&);
  /// Private assignment operator (this class is a singleton)
  registrar& operator=(const registrar&);

  /// Print a message when in debug mode
  void debug_msg( std::string msg ) const;
};


} // end namespace maptk


#endif // MAPTK_REGISTRAR_H_
