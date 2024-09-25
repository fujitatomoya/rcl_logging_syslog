// Copyright 2024 Tomoya Fujita <tomoya.fujita825@gmail.com>.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "gmock/gmock.h"

#include "rcl_logging_interface/rcl_logging_interface.h"

#include "rcpputils/env.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rcutils/allocator.h"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"
#include "rcutils/process.h"
#include "rcutils/strdup.h"
#include "rcutils/testing/fault_injection.h"

static constexpr int logger_levels[] =
{
  RCUTILS_LOG_SEVERITY_UNSET,
  RCUTILS_LOG_SEVERITY_DEBUG,
  RCUTILS_LOG_SEVERITY_INFO,
  RCUTILS_LOG_SEVERITY_WARN,
  RCUTILS_LOG_SEVERITY_ERROR,
  RCUTILS_LOG_SEVERITY_FATAL,
};

// This is a helper class that resets an environment
// variable when leaving scope
class RestoreEnvVar final
{
public:
  explicit RestoreEnvVar(const std::string & name)
  : name_(name),
    value_(rcpputils::get_env_var(name.c_str()))
  {
  }

  ~RestoreEnvVar()
  {
    if (!rcpputils::set_env_var(name_.c_str(), value_.c_str())) {
      std::cerr << "Failed to restore value of environment variable: " << name_ << std::endl;
    }
  }

private:
  const std::string name_;
  const std::string value_;
};

class AllocatorTest : public ::testing::Test
{
public:
  AllocatorTest()
  : allocator(rcutils_get_default_allocator()),
    bad_allocator(get_bad_allocator()),
    invalid_allocator(rcutils_get_zero_initialized_allocator())
  {
  }

  rcutils_allocator_t allocator;
  rcutils_allocator_t bad_allocator;
  rcutils_allocator_t invalid_allocator;

private:
  static rcutils_allocator_t get_bad_allocator()
  {
    rcutils_allocator_t bad_allocator = rcutils_get_default_allocator();
    bad_allocator.allocate = AllocatorTest::bad_malloc;
    bad_allocator.reallocate = AllocatorTest::bad_realloc;
    return bad_allocator;
  }

  static void * bad_malloc(size_t, void *)
  {
    return nullptr;
  }

  static void * bad_realloc(void *, size_t, void *)
  {
    return nullptr;
  }
};

TEST_F(AllocatorTest, init_invalid)
{
  EXPECT_EQ(
    RCL_LOGGING_RET_ERROR,
    rcl_logging_external_initialize("file_name_prefix", nullptr, bad_allocator));
  rcutils_reset_error();
  EXPECT_EQ(
    RCL_LOGGING_RET_INVALID_ARGUMENT,
    rcl_logging_external_initialize(nullptr, nullptr, invalid_allocator));
  rcutils_reset_error();
}

TEST_F(AllocatorTest, init_failure)
{
  RestoreEnvVar home_var("HOME");
  RestoreEnvVar userprofile_var("USERPROFILE");

  // No home directory to write log to
  ASSERT_TRUE(rcpputils::set_env_var("HOME", nullptr));
  ASSERT_TRUE(rcpputils::set_env_var("USERPROFILE", nullptr));
  EXPECT_EQ(RCL_LOGGING_RET_ERROR, rcl_logging_external_initialize(nullptr, nullptr, allocator));
  rcutils_reset_error();
}

TEST_F(AllocatorTest, init_valid)
{
  // This success test must be placed in the end after failure cases
  // with test fixture class AllocatorTest. Because this succeeds,
  // syslog log facility is allocated, this means init returns always success.

  // Config files are not supported, and pass through with warning.
  EXPECT_EQ(
    RCL_LOGGING_RET_OK,
    rcl_logging_external_initialize(nullptr, "config_file", allocator));
  rcutils_reset_error();
}
