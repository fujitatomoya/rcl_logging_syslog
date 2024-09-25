cmake_minimum_required(VERSION 3.5)

project(rcl_logging_syslog)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rcl_logging_interface REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wformat=2 -Wconversion -Woverloaded-virtual -Wshadow
    -Wnon-virtual-dtor -Wold-style-cast -Wcast-qual
  )
endif()

add_library(${PROJECT_NAME} src/rcl_logging_syslog.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
  rcpputils::rcpputils
  rcutils::rcutils)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rcl_logging_interface::rcl_logging_interface)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RCL_LOGGING_INTERFACE_BUILDING_DLL")

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cppcheck 1.90 doesn't understand some of the syntax in spdlog's bundled fmt
  # code.  Since we already have cppcheck disabled on Linux, just disable it
  # completely for this package.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gmock(test_logging_interface test/test_logging_interface.cpp)
  if(TARGET test_logging_interface)
    target_link_libraries(test_logging_interface ${PROJECT_NAME} rcpputils::rcpputils)
    target_compile_definitions(test_logging_interface PUBLIC RCUTILS_ENABLE_FAULT_INJECTION)
  endif()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_package()
