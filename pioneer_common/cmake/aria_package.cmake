#
# Find Aria / AriaCoda and set up include directories
#
# @public
#
macro(include_aria_directories)
  find_package(Aria QUIET)

  if(Aria_FOUND)
    message("CMake package for Aria was found, using that.")

    # Aria package for CMake was found
    if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
      add_definitions(-DADEPT_PKG)
    endif()

    include_directories(${Aria_INCLUDE_DIRS})
    link_directories(${Aria_LIBRARY_DIRS})
  else()
    # No Aria package for CMake was found, search ourselves
    # The installation package provided by Adept doesn't follow normal Linux
    # installation locations, but the repackaged Debian package and AriaCoda do.
    # If user set ARIA environment variable, look there, otherwise look in
    # /usr/local/.
    if(DEFINED ENV{ARIA})
      message("ARIA environment variable defined, checking there...")
      set(prefix $ENV{ARIA})
    else()
      set(prefix "/usr/local")
    endif()

    message("Looking for Aria in ${prefix}/Aria and ${prefix}...")

    if(EXISTS "${prefix}/Aria/include/Aria.h")
      message("Found ${prefix}/Aria/include/Aria.h, assuming Adept ARIA package.")
      add_definitions(-DADEPT_PKG)
      include_directories(${prefix}/Aria/include)
      link_directories(${prefix}/Aria/lib)
    else()
      if(EXISTS "${prefix}/include/Aria.h")
        message("Found ${prefix)/include/Aria.h, assuming Adept ARIA source directory.")
        add_definitions(-DADEPT_PKG)
        include_directories(${prefix}/include)
        link_directories(${prefix}/lib)
      else()
        if(EXISTS "${prefix}/include/Aria/Aria.h")
          message("Found ${prefix}/include/Aria/Aria.h, assuming AriaCoda or repackaged ARIA.")

          # add_definitions(-DARIACODA)
          include_directories(${prefix}/include)
          link_directories(${prefix}/lib)
        else()
          message("Aria.h not found in ${prefix}. Continuing with default header and library paths.")
        endif()
      endif()
    endif()
  endif()
endmacro()

# #############################################################################

#
# Target link for Aria libraries
#
# @public
#
macro(target_link_aria_libraries target)
  target_link_libraries(${target} PUBLIC Aria)
endmacro()