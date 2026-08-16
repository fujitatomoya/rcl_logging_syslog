// Copyright 2026 Tomoya Fujita <tomoya.fujita825@gmail.com>.
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

// Test-only translation unit for ros2-abi-action verification.
//
// This redefines rcl_logging_external_log with a modified parameter type
// (int severity -> long severity). It deliberately does NOT include
// rcl_logging_interface.h so the conflicting declaration is not seen by the
// compiler. The exported ELF symbol name is unchanged (C linkage), but the
// DWARF signature differs, which abidiff must detect as an incompatible
// ABI change.

#include <syslog.h>

extern "C" void rcl_logging_external_log(long severity, const char * name, const char * msg)
{
  (void) name;
  (void) severity;
  syslog(LOG_INFO, "%s", msg);
}
