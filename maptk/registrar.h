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
 * \brief algorithm regstrar class
 *
 * Not exported as its used internally as an implementation detail.
 */

#ifndef MAPTK_REGISTRAR_H_
#define MAPTK_REGISTRAR_H_

#include <string>
#include <map>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

namespace maptk
{

/// A singleton class to keep a registry of objects of type T
template <typename T>
class registrar
{
public:
  /// map type for string-to-item relationship
  typedef std::map<std::string, boost::shared_ptr<T> > item_map;
  /// convenience type for item_map's values
  typedef typename item_map::value_type item_pair;
  /// convenienve type for item_map's const iterator
  typedef typename item_map::const_iterator item_const_itr;

  /// Access the singleton instance of this class
  static registrar<T>& instance()
  {
    if (!instance_)
    {
      create_instance();
    }
    return *instance_;
  }

  /// Register a new name and item with the registrar
  static bool register_item(const std::string& name, boost::shared_ptr<T> item)
  {
    item_const_itr it = instance().registry_.find(name);
    if (it == instance().registry_.end())
    {
      instance().registry_.insert(item_pair(name,item));
    }
    else
    {
      std::cerr << "Warning: duplicate registration of \""
                << name << "\"" << std::endl;
      return false;
    }
    return true;
  }

  /// Return a vector of registered item names
  static std::vector<std::string> registered_names()
  {
    std::vector<std::string> names;
    BOOST_FOREACH(item_pair i, instance().registry_)
    {
      names.push_back(i.first);
    }
    return names;
  }

  /// Find the item matching \a name or return NULL
  static boost::shared_ptr<T> find(const std::string& name)
  {
    item_const_itr it = instance().registry_.find(name);
    if (it == instance().registry_.end())
    {
      return boost::shared_ptr<T>();
    }
    return it->second;
  }

private:
  /// The registry of items
  item_map registry_;

  /// The pointer to the singleton instance
  static registrar<T>* instance_;

  /// Create the singleton instance
  static void create_instance()
  {
    static registrar<T> inst;
    instance_ = &inst;
  }

  /// Private constructor (this class is a singleton)
  registrar<T>() {}
  /// Private destructor (this class is a singleton)
  ~registrar<T>() {}
  /// Private copy constructor (this class is a singleton)
  registrar<T>(const registrar<T>& );
  /// Private assignment operator (this class is a singleton)
  registrar<T>& operator=(const registrar<T>&);
};


} // end namespace maptk


#endif // MAPTK_REGISTRAR_H_
