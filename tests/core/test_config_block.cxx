/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/config_block.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

IMPLEMENT_TEST(block_sep_size)
{
  if (maptk::config_block::block_sep.size() != 1)
  {
    TEST_ERROR("Block separator is not size 1; quite a "
               "few places rest on this assumption now");
  }
}

IMPLEMENT_TEST(has_value)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(keya, valuea);

  if (!config->has_value(keya))
  {
    TEST_ERROR("Block does not have value which was set");
  }

  if (config->has_value(keyb))
  {
    TEST_ERROR("Block has value which was not set");
  }
}

IMPLEMENT_TEST(get_value)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(keya, valuea);

  maptk::config_block_value_t const get_valuea = config->get_value<maptk::config_block_value_t>(keya);

  if (valuea != get_valuea)
  {
    TEST_ERROR("Did not retrieve value that was set");
  }
}

IMPLEMENT_TEST(get_value_nested)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(keya + maptk::config_block::block_sep + keyb, valuea);

  maptk::config_block_t const nested_config = config->subblock(keya);

  maptk::config_block_value_t const get_valuea = nested_config->get_value<maptk::config_block_value_t>(keyb);

  if (valuea != get_valuea)
  {
    TEST_ERROR("Did not retrieve value that was set");
  }
}

IMPLEMENT_TEST(get_value_no_exist)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");

  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");

  EXPECT_EXCEPTION(maptk::no_such_configuration_value_exception,
                   config->get_value<maptk::config_block_value_t>(keya),
                   "retrieving an unset value");

  maptk::config_block_value_t const get_valueb = config->get_value<maptk::config_block_value_t>(keyb, valueb);

  if (valueb != get_valueb)
  {
    TEST_ERROR("Did not retrieve default when requesting unset value");
  }
}

IMPLEMENT_TEST(get_value_type_mismatch)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  int const valueb = 100;

  config->set_value(keya, valuea);

  EXPECT_EXCEPTION(maptk::bad_config_block_cast_exception,
                   config->get_value<int>(keya),
                   "doing an invalid cast");

  int const get_valueb = config->get_value<int>(keya, valueb);

  if (valueb != get_valueb)
  {
    TEST_ERROR("Did not retrieve default when requesting a bad cast");
  }
}

IMPLEMENT_TEST(bool_conversion)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const key = maptk::config_block_key_t("key");

  maptk::config_block_value_t const lit_true = maptk::config_block_value_t("true");
  maptk::config_block_value_t const lit_false = maptk::config_block_value_t("false");
  maptk::config_block_value_t const lit_True = maptk::config_block_value_t("True");
  maptk::config_block_value_t const lit_False = maptk::config_block_value_t("False");
  maptk::config_block_value_t const lit_1 = maptk::config_block_value_t("1");
  maptk::config_block_value_t const lit_0 = maptk::config_block_value_t("0");

  bool val;

  config->set_value(key, lit_true);
  val = config->get_value<bool>(key);

  if (!val)
  {
    TEST_ERROR("The value \'true\' did not get converted to true when read as a boolean");
  }

  config->set_value(key, lit_false);
  val = config->get_value<bool>(key);

  if (val)
  {
    TEST_ERROR("The value \'false\' did not get converted to false when read as a boolean");
  }

  config->set_value(key, lit_True);
  val = config->get_value<bool>(key);

  if (!val)
  {
    TEST_ERROR("The value \'True\' did not get converted to true when read as a boolean");
  }

  config->set_value(key, lit_False);
  val = config->get_value<bool>(key);

  if (val)
  {
    TEST_ERROR("The value \'False\' did not get converted to false when read as a boolean");
  }

  config->set_value(key, lit_1);
  val = config->get_value<bool>(key);

  if (!val)
  {
    TEST_ERROR("The value \'1\' did not get converted to true when read as a boolean");
  }

  config->set_value(key, lit_0);
  val = config->get_value<bool>(key);

  if (val)
  {
    TEST_ERROR("The value \'0\' did not get converted to true when read as a boolean");
  }
}

IMPLEMENT_TEST(unset_value)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");

  config->set_value(keya, valuea);
  config->set_value(keyb, valueb);

  config->unset_value(keya);

  EXPECT_EXCEPTION(maptk::no_such_configuration_value_exception,
                   config->get_value<maptk::config_block_value_t>(keya),
                   "retrieving an unset value");

  maptk::config_block_value_t const get_valueb = config->get_value<maptk::config_block_value_t>(keyb);

  if (valueb != get_valueb)
  {
    TEST_ERROR("Did not retrieve value when requesting after an unrelated unset");
  }
}

IMPLEMENT_TEST(available_values)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");

  config->set_value(keya, valuea);
  config->set_value(keyb, valueb);

  maptk::config_block_keys_t keys;

  keys.push_back(keya);
  keys.push_back(keyb);

  maptk::config_block_keys_t const get_keys = config->available_values();

  if (keys.size() != get_keys.size())
  {
    TEST_ERROR("Did not retrieve correct number of keys");
  }
}

IMPLEMENT_TEST(read_only)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");

  config->set_value(keya, valuea);

  config->mark_read_only(keya);

  EXPECT_EXCEPTION(maptk::set_on_read_only_value_exception,
                   config->set_value(keya, valueb),
                   "setting a read only value");

  maptk::config_block_value_t const get_valuea = config->get_value<maptk::config_block_value_t>(keya);

  if (valuea != get_valuea)
  {
    TEST_ERROR("Read only value changed");
  }
}

IMPLEMENT_TEST(read_only_unset)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(keya, valuea);

  config->mark_read_only(keya);

  EXPECT_EXCEPTION(maptk::unset_on_read_only_value_exception,
                   config->unset_value(keya),
                   "unsetting a read only value");

  maptk::config_block_value_t const get_valuea = config->get_value<maptk::config_block_value_t>(keya);

  if (valuea != get_valuea)
  {
    TEST_ERROR("Read only value was unset");
  }
}

IMPLEMENT_TEST(subblock)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");
  maptk::config_block_key_t const other_block_name = maptk::config_block_key_t("other_block");

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");
  maptk::config_block_key_t const keyc = maptk::config_block_key_t("keyc");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");
  maptk::config_block_value_t const valuec = maptk::config_block_value_t("valuec");

  config->set_value(block_name + maptk::config_block::block_sep + keya, valuea);
  config->set_value(block_name + maptk::config_block::block_sep + keyb, valueb);
  config->set_value(other_block_name + maptk::config_block::block_sep + keyc, valuec);

  maptk::config_block_t const subblock = config->subblock(block_name);

  if (subblock->has_value(keya))
  {
    maptk::config_block_value_t const get_valuea = subblock->get_value<maptk::config_block_value_t>(keya);

    if (valuea != get_valuea)
    {
      TEST_ERROR("Subblock did not inherit expected keys");
    }
  }
  else
  {
    TEST_ERROR("Subblock did not inherit expected keys");
  }

  if (subblock->has_value(keyb))
  {
    maptk::config_block_value_t const get_valueb = subblock->get_value<maptk::config_block_value_t>(keyb);

    if (valueb != get_valueb)
    {
      TEST_ERROR("Subblock did not inherit expected keys");
    }
  }
  else
  {
    TEST_ERROR("Subblock did not inherit expected keys");
  }

  if (subblock->has_value(keyc))
  {
    TEST_ERROR("Subblock inherited unrelated key");
  }
}

IMPLEMENT_TEST(subblock_nested)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");
  maptk::config_block_key_t const other_block_name = maptk::config_block_key_t("other_block");
  maptk::config_block_key_t const nested_block_name = block_name + maptk::config_block::block_sep + other_block_name;

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");

  config->set_value(nested_block_name + maptk::config_block::block_sep + keya, valuea);
  config->set_value(nested_block_name + maptk::config_block::block_sep + keyb, valueb);

  maptk::config_block_t const subblock = config->subblock(nested_block_name);

  if (subblock->has_value(keya))
  {
    maptk::config_block_value_t const get_valuea = subblock->get_value<maptk::config_block_value_t>(keya);

    if (valuea != get_valuea)
    {
      TEST_ERROR("Nested subblock did not inherit expected keys");
    }
  }
  else
  {
    TEST_ERROR("Subblock did not inherit expected keys");
  }

  if (subblock->has_value(keyb))
  {
    maptk::config_block_value_t const get_valueb = subblock->get_value<maptk::config_block_value_t>(keyb);

    if (valueb != get_valueb)
    {
      TEST_ERROR("Nested subblock did not inherit expected keys");
    }
  }
  else
  {
    TEST_ERROR("Subblock did not inherit expected keys");
  }
}

IMPLEMENT_TEST(subblock_match)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(block_name, valuea);

  maptk::config_block_t const subblock = config->subblock(block_name);

  maptk::config_block_keys_t const keys = subblock->available_values();

  if (!keys.empty())
  {
    TEST_ERROR("A subblock inherited a value that shared the block name");
  }
}

IMPLEMENT_TEST(subblock_prefix_match)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(block_name + keya, valuea);

  maptk::config_block_t const subblock = config->subblock(block_name);

  maptk::config_block_keys_t const keys = subblock->available_values();

  if (!keys.empty())
  {
    TEST_ERROR("A subblock inherited a value that shared a prefix with the block name");
  }
}

IMPLEMENT_TEST(subblock_view)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");
  maptk::config_block_key_t const other_block_name = maptk::config_block_key_t("other_block");

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");
  maptk::config_block_key_t const keyc = maptk::config_block_key_t("keyc");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");
  maptk::config_block_value_t const valuec = maptk::config_block_value_t("valuec");

  config->set_value(block_name + maptk::config_block::block_sep + keya, valuea);
  config->set_value(block_name + maptk::config_block::block_sep + keyb, valueb);
  config->set_value(other_block_name + maptk::config_block::block_sep + keyc, valuec);

  maptk::config_block_t const subblock = config->subblock_view(block_name);

  if (!subblock->has_value(keya))
  {
    TEST_ERROR("Subblock view did not inherit key");
  }

  if (subblock->has_value(keyc))
  {
    TEST_ERROR("Subblock view inherited unrelated key");
  }

  config->set_value(block_name + maptk::config_block::block_sep + keya, valueb);

  maptk::config_block_value_t const get_valuea1 = subblock->get_value<maptk::config_block_value_t>(keya);

  if (valueb != get_valuea1)
  {
    TEST_ERROR("Subblock view persisted a changed value");
  }

  subblock->set_value(keya, valuea);

  maptk::config_block_value_t const get_valuea2 = config->get_value<maptk::config_block_value_t>(block_name + maptk::config_block::block_sep + keya);

  if (valuea != get_valuea2)
  {
    TEST_ERROR("Subblock view set value was not changed in parent");
  }

  subblock->unset_value(keyb);

  if (config->has_value(block_name + maptk::config_block::block_sep + keyb))
  {
    TEST_ERROR("Unsetting from a subblock view did not unset in parent view");
  }

  config->set_value(block_name + maptk::config_block::block_sep + keyc, valuec);

  maptk::config_block_keys_t keys;

  keys.push_back(keya);
  keys.push_back(keyc);

  maptk::config_block_keys_t const get_keys = subblock->available_values();

  if (keys.size() != get_keys.size())
  {
    TEST_ERROR("Did not retrieve correct number of keys from the subblock");
  }
}

IMPLEMENT_TEST(subblock_view_nested)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");
  maptk::config_block_key_t const other_block_name = maptk::config_block_key_t("other_block");
  maptk::config_block_key_t const nested_block_name = block_name + maptk::config_block::block_sep + other_block_name;

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");

  config->set_value(nested_block_name + maptk::config_block::block_sep + keya, valuea);
  config->set_value(nested_block_name + maptk::config_block::block_sep + keyb, valueb);

  maptk::config_block_t const subblock = config->subblock_view(nested_block_name);

  if (subblock->has_value(keya))
  {
    maptk::config_block_value_t const get_valuea = subblock->get_value<maptk::config_block_value_t>(keya);

    if (valuea != get_valuea)
    {
      TEST_ERROR("Nested subblock did not inherit expected keys");
    }
  }
  else
  {
    TEST_ERROR("Subblock did not inherit expected keys");
  }

  if (subblock->has_value(keyb))
  {
    maptk::config_block_value_t const get_valueb = subblock->get_value<maptk::config_block_value_t>(keyb);

    if (valueb != get_valueb)
    {
      TEST_ERROR("Nested subblock did not inherit expected keys");
    }
  }
  else
  {
    TEST_ERROR("Subblock did not inherit expected keys");
  }
}

IMPLEMENT_TEST(subblock_view_match)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(block_name, valuea);

  maptk::config_block_t const subblock = config->subblock_view(block_name);

  maptk::config_block_keys_t const keys = subblock->available_values();

  if (!keys.empty())
  {
    TEST_ERROR("A subblock inherited a value that shared the block name");
  }
}

IMPLEMENT_TEST(subblock_view_prefix_match)
{
  maptk::config_block_t const config = maptk::config_block::empty_config();

  maptk::config_block_key_t const block_name = maptk::config_block_key_t("block");

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");

  config->set_value(block_name + keya, valuea);

  maptk::config_block_t const subblock = config->subblock_view(block_name);

  maptk::config_block_keys_t const keys = subblock->available_values();

  if (!keys.empty())
  {
    TEST_ERROR("A subblock inherited a value that shared a prefix with the block name");
  }
}

IMPLEMENT_TEST(merge_config)
{
  maptk::config_block_t const configa = maptk::config_block::empty_config();
  maptk::config_block_t const configb = maptk::config_block::empty_config();

  maptk::config_block_key_t const keya = maptk::config_block_key_t("keya");
  maptk::config_block_key_t const keyb = maptk::config_block_key_t("keyb");
  maptk::config_block_key_t const keyc = maptk::config_block_key_t("keyc");

  maptk::config_block_value_t const valuea = maptk::config_block_value_t("valuea");
  maptk::config_block_value_t const valueb = maptk::config_block_value_t("valueb");
  maptk::config_block_value_t const valuec = maptk::config_block_value_t("valuec");

  configa->set_value(keya, valuea);
  configa->set_value(keyb, valuea);

  configb->set_value(keyb, valueb);
  configb->set_value(keyc, valuec);

  configa->merge_config(configb);

  maptk::config_block_value_t const get_valuea = configa->get_value<maptk::config_block_value_t>(keya);

  if (valuea != get_valuea)
  {
    TEST_ERROR("Unmerged key changed");
  }

  maptk::config_block_value_t const get_valueb = configa->get_value<maptk::config_block_value_t>(keyb);

  if (valueb != get_valueb)
  {
    TEST_ERROR("Conflicting key was not overwritten");
  }

  maptk::config_block_value_t const get_valuec = configa->get_value<maptk::config_block_value_t>(keyc);

  if (valuec != get_valuec)
  {
    TEST_ERROR("New key did not appear");
  }
}
