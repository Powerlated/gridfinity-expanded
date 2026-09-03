cmake_minimum_required(VERSION 3.25)
project(gridfinity_occt_vendor LANGUAGES C CXX)

include(cmake/occt-common.cmake)
# OCCT assumes it is the top-level project when adding its generated include
# directory. In this wrapper the generated headers live below its binary dir.
include_directories("${CMAKE_BINARY_DIR}/vendor/occt/${INSTALL_DIR_INCLUDE}")
add_subdirectory(vendor/occt)
# OCCT's install rule also assumes its generated launcher is top-level.
if(WIN32)
  set(GRIDFINITY_SCRIPT_EXT bat)
else()
  set(GRIDFINITY_SCRIPT_EXT sh)
endif()
configure_file(
  "${CMAKE_BINARY_DIR}/vendor/occt/env.${GRIDFINITY_SCRIPT_EXT}"
  "${CMAKE_BINARY_DIR}/env.${GRIDFINITY_SCRIPT_EXT}"
  COPYONLY
)
foreach(_occt_config
    OpenCASCADECompileDefinitionsAndFlags-debug.cmake
    OpenCASCADECompileDefinitionsAndFlags-release.cmake
    OpenCASCADECompileDefinitionsAndFlags-relwithdebinfo.cmake
    OpenCASCADEConfig.cmake)
  configure_file(
    "${CMAKE_BINARY_DIR}/vendor/occt/${_occt_config}"
    "${CMAKE_BINARY_DIR}/${_occt_config}"
    COPYONLY
  )
endforeach()
