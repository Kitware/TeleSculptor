/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "config_io.h"

#include <fstream>
#include <iostream>
#include <vector>

#include <maptk/core/exceptions.h>

#include <boost/filesystem.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi.hpp>

namespace qi = boost::spirit::qi;

namespace maptk
{

struct config_value_s {
  /// the configuration path within the block structure
  config_key_t key;
  /// value associated with the given key
  config_value_t value;
};

/// The type that represents multiple stored configuration values.
typedef std::vector<config_value_s> config_value_set;

}

// Adapting structure to boost fusion tuples for use in spirit::qi
BOOST_FUSION_ADAPT_STRUCT(
  maptk::config_value_s,
  (maptk::config_key_t, key)
  (maptk::config_value_t, value)
)

namespace maptk
{

namespace
{

static token_t const config_block_start = token_t("block");
static token_t const config_block_end = token_t("endblock");

template<typename Iterator>
class config_grammar
  : public qi::grammar<Iterator, config_value_set()>
{
  public:
    config_grammar();
    ~config_grammar();

  private:
    qi::rule<Iterator> opt_whitespace;
    qi::rule<Iterator> whitespace;
    qi::rule<Iterator> eol;
    qi::rule<Iterator> line_end;

    qi::rule<Iterator, config_key_t()> config_key;
    qi::rule<Iterator, config_value_t()> config_value;
    qi::rule<Iterator, config_value_s()> config_value_full;
    qi::rule<Iterator, config_value_set()> config_values;
};

template <typename Iterator>
config_grammar<Iterator>
::config_grammar()
  : opt_whitespace()
  , whitespace()
  , eol()
  , line_end()
  , config_key()
  , config_value()
  , config_value_full()
  , config_values()
  , config_grammar::base_type(config_values, "config-declaration")
{
  opt_whitespace.name("opt-whitespace");
  opt_whitespace %= *( qi::blank );

  whitespace.name("whitespace");
  whitespace %= +( qi::blank );

  eol.name("eol");
  eol %= qi::lit("\r\n")
       | qi::lit("\n");

  line_end.name("line-end");
  line_end %= +( eol );

  config_key.name("config-key");
  config_key %=
    +( qi::alnum
     | qi::char_("_")
     );

  config_value.name("config-value");
  config_value %=
    +( qi::graph
     | qi::char_(' ')
     | qi::char_('\t')
     );

  config_value_full.name("config-value-full");
  config_value_full %=
    (
      opt_whitespace >> config_key > opt_whitespace > '=' > opt_whitespace > config_value > line_end
    );

  config_values.name("config-values");
  config_values %= +config_value_full;

}

}

/// Read in a configuration file, producing a \c config object.
config_t read_config_file(path_t const& file_path)
{
  // Check that file exists
  if( ! boost::filesystem::exists(file_path) )
  {
    throw file_not_found_exception(file_path, "File does not exist.");
  }
  else if ( ! boost::filesystem::is_regular_file(file_path) )
  {
    throw file_not_found_exception(file_path, "Path given doesn't point to "
                                              "a regular file!");
  }

  // Reading in input file data
  std::fstream input_stream(file_path.c_str(), std::fstream::in);
  if( ! input_stream )
  {
    throw file_not_read_exception(file_path, "Could not open file at given "
                                             "path.");
  }
  std::string storage;
  input_stream.unsetf(std::ios::skipws);
  std::copy(std::istream_iterator<char>(input_stream),
            std::istream_iterator<char>(),
            std::back_inserter(storage));

  // Commence parsing!
  config_value_set config_values;
  config_grammar grammar;
  std::string::const_iterator s_begin = storage.begin();
  std::string::const_iterator s_end = storage.end();
  bool r = qi::parse(s_begin, s_end, grammar, config_values);

  if( r && s_begin == s_end )
  {
    std::cerr << "File parsed! Contained " << config_values.size() << " entries." << std::endl;
  }
  else if (!r)
  {
    throw file_not_read_exception(file_path, "File not parsed, or parsed completely!");
  }
}

}
