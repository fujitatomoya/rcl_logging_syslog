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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include <memory>
#include <system_error>

#include "rcpputils/env.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rcutils/allocator.h"
#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"
#include "rcutils/process.h"
#include "rcutils/snprintf.h"
#include "rcutils/strdup.h"
#include "rcutils/time.h"

#include "rcl_logging_interface/rcl_logging_interface.h"

static const char * facility_env_name = "RCL_LOGGING_SYSLOG_FACILITY";

// see https://codebrowser.dev/glibc/glibc/misc/syslog.c.html#LogTag
// This memory needs to be kept until closelog()
static std::shared_ptr<std::string> syslog_identity;

static const char *logger_name = "rcl_logging_syslog";

typedef struct facility_index
{
  const char *c_name;
  const int c_value;
} FACILITY_INDEX;

const FACILITY_INDEX facility_table[] =
{
  {"LOG_CRON", LOG_CRON},
  {"LOG_DAEMON", LOG_DAEMON},
  {"LOG_SYSLOG", LOG_SYSLOG},
  {"LOG_USER", LOG_USER},
  {"LOG_LOCAL0", LOG_LOCAL0},
  {"LOG_LOCAL1", LOG_LOCAL1},
  {"LOG_LOCAL2", LOG_LOCAL2},
  {"LOG_LOCAL3", LOG_LOCAL3},
  {"LOG_LOCAL4", LOG_LOCAL4},
  {"LOG_LOCAL5", LOG_LOCAL5},
  {"LOG_LOCAL6", LOG_LOCAL6},
  {"LOG_LOCAL7", LOG_LOCAL7},
  {NULL, -1}
};

static const char * get_facility_name(int facility)
{
  for (auto f = facility_table; f->c_name != NULL; f++) {
    if (f->c_value == facility) {
      return f->c_name;
    }
  }
  return "Unknown facility";
}

static int get_syslog_facility(void)
{
  // default is LOG_LOCAL1
  int syslog_facility = LOG_LOCAL1;

  try {
    std::string facility_string = rcpputils::get_env_var(facility_env_name);
    if (facility_string.empty()) {
      return syslog_facility;
    }

    if (!facility_string.compare("LOG_USER")) {
      syslog_facility = LOG_USER;
    } else if (!facility_string.compare("LOG_DAEMON")) {
      syslog_facility = LOG_DAEMON;
    } else if (!facility_string.compare("LOG_SYSLOG")) {
      syslog_facility = LOG_SYSLOG;
    } else if (!facility_string.compare("LOG_CRON")) {
      syslog_facility = LOG_CRON;
    } else if (!facility_string.compare("LOG_LOCAL0")) {
      syslog_facility = LOG_LOCAL0;
    } else if (!facility_string.compare("LOG_LOCAL1")) {
      syslog_facility = LOG_LOCAL1;
    } else if (!facility_string.compare("LOG_LOCAL2")) {
      syslog_facility = LOG_LOCAL2;
    } else if (!facility_string.compare("LOG_LOCAL3")) {
      syslog_facility = LOG_LOCAL3;
    } else if (!facility_string.compare("LOG_LOCAL4")) {
      syslog_facility = LOG_LOCAL4;
    } else if (!facility_string.compare("LOG_LOCAL5")) {
      syslog_facility = LOG_LOCAL5;
    } else if (!facility_string.compare("LOG_LOCAL6")) {
      syslog_facility = LOG_LOCAL6;
    } else if (!facility_string.compare("LOG_LOCAL7")) {
      syslog_facility = LOG_LOCAL7;
    } else {
      syslog_facility = LOG_LOCAL1;
    }
  } catch (const std::runtime_error & error) {
    throw std::runtime_error(
            std::string("failed to get env var '") + facility_env_name + "': " + error.what()
    );
  }

  return syslog_facility;
}

static int rcutil_to_syslog_level(int rcutil_level)
{
  int syslog_level;

  if (rcutil_level <= RCUTILS_LOG_SEVERITY_DEBUG) {
    syslog_level = LOG_DEBUG;
  } else if (rcutil_level <= RCUTILS_LOG_SEVERITY_INFO) {
    syslog_level = LOG_INFO;
  } else if (rcutil_level <= RCUTILS_LOG_SEVERITY_WARN) {
    syslog_level = LOG_WARNING;
  } else if (rcutil_level <= RCUTILS_LOG_SEVERITY_ERROR) {
    syslog_level = LOG_ERR;
  } else if (rcutil_level <= RCUTILS_LOG_SEVERITY_FATAL) {
    syslog_level = LOG_ALERT;
  }
  return syslog_level;
}

[[maybe_unused]] static void vlog_msg(int level, const char *format, ...)
{
  va_list args;
  va_start(args, format);
  vsyslog(level, format, args);
  va_end(args);
}

rcl_logging_ret_t rcl_logging_external_initialize(
  const char * file_name_prefix,
  const char * config_file,
  rcutils_allocator_t allocator)
{
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RCL_LOGGING_RET_INVALID_ARGUMENT);

  // It is possible for this to get called more than once in a process (some of
  // the tests do this implicitly by calling rclcpp::init more than once).
  // If the logger is already setup, don't do anything.
  if (syslog_identity != nullptr) {
    return RCL_LOGGING_RET_OK;
  }

  bool config_file_provided = (nullptr != config_file) && (config_file[0] != '\0');
  if (config_file_provided) {
    // rsyslog does not have much configuration for user API, most of the configuration
    // needs to be set in /etc/rsyslog.conf file instead.
    RCUTILS_LOG_WARN_NAMED(
      logger_name,
      "syslog logging backend doesn't have client configuration, use rsyslogd.conf instead");
  }

  char * logdir = nullptr;
  rcl_logging_ret_t dir_ret = rcl_logging_get_logging_directory(allocator, &logdir);
  if (RCL_LOGGING_RET_OK != dir_ret) {
    RCUTILS_SET_ERROR_MSG_AND_APPEND_PREV_ERROR("Failed to get logging directory");
    return dir_ret;
  } else {
    // rsyslog does not allow user to create or specify logging directory via API.
    // Most of the configuration needs to be set in /etc/rsyslog.conf file instead.
    RCUTILS_LOG_DEBUG_NAMED(
      logger_name,
      "syslog logging backend doesn't support external configuration, "
      "configure rsyslogd.conf instead of %s log directory", logdir);
  }
  RCPPUTILS_SCOPE_EXIT(
  {
    allocator.deallocate(logdir, allocator.state);
  });

  RCUTILS_LOG_DEBUG_NAMED(
    logger_name,
    "executable or file name will be used for openlog(*identification), "
    "to change the log file name on file system, configure /etc/rsyslogd.conf instead");

  bool file_name_provided = (nullptr != file_name_prefix) && (file_name_prefix[0] != '\0');
  char * basec;
  if (file_name_provided) {
    basec = rcutils_strdup(file_name_prefix, allocator);
  } else {  // otherwise, get the program name.
    basec = rcutils_get_executable_name(allocator);
  }
  if (basec == nullptr) {
    RCUTILS_LOG_ERROR("Failed to get the executable name");
    return RCL_LOGGING_RET_ERROR;
  }
  RCPPUTILS_SCOPE_EXIT(
  {
    allocator.deallocate(basec, allocator.state);
  });
  syslog_identity = std::make_shared<std::string>(basec);

  // Check and fetch the syslog facility from environmental variable
  int syslog_facility = get_syslog_facility();
  RCUTILS_LOG_DEBUG_NAMED(
    logger_name,
    "syslog facility is set to %s", get_facility_name(syslog_facility));

  // Use user specified filename, or executable name to openlog(3) identity.
  openlog(syslog_identity->c_str(), LOG_PID, syslog_facility);

  return RCL_LOGGING_RET_OK;
}

rcl_logging_ret_t rcl_logging_external_shutdown()
{
  closelog();
  syslog_identity = nullptr;
  return RCL_LOGGING_RET_OK;
}

void rcl_logging_external_log(int severity, const char * name, const char * msg)
{
  (void) name;
  syslog(rcutil_to_syslog_level(severity), "%s", msg);
}

rcl_logging_ret_t rcl_logging_external_set_logger_level(const char * name, int level)
{
  (void) name;

  int syslog_level = rcutil_to_syslog_level(level);
  int old_mask = setlogmask(LOG_UPTO(syslog_level));
  if (old_mask == -1) {
    RCUTILS_LOG_ERROR("Failed to call setlogmask(%d)", syslog_level);
    return RCL_LOGGING_RET_ERROR;
  }

  return RCL_LOGGING_RET_OK;
}
