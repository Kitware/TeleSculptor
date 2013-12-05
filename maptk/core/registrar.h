/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_REGISTRAR_H_
#define MAPTK_REGISTRAR_H_

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

namespace maptk
{

/// A singleton class to keep a registry of objects of type T
template <typename T>
class registrar
{
public:
  typedef std::map<std::string, boost::shared_ptr<T> > item_map;
  typedef typename item_map::value_type item_pair;
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
