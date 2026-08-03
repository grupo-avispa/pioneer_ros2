#
# Apply the common build type, C++ standard, warning flags, coverage option and
# MSVC defaults shared by every package in the pioneer_ros2 stack.
#
# @public
#
macro(pioneer_default_build_settings)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
      STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  endif()

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
      set(CMAKE_CXX_STANDARD 17)
    else()
      message(FATAL_ERROR "cxx_std_17 could not be found.")
    endif()
  endif()

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
  endif()

  option(COVERAGE_ENABLED "Enable code coverage" FALSE)

  if(COVERAGE_ENABLED)
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
  endif()

  # Defaults for Microsoft C++ compiler
  if(MSVC)
    # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

    # Enable Math Constants
    # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
    add_compile_definitions(
      _USE_MATH_DEFINES
    )
  endif()
endmacro()
