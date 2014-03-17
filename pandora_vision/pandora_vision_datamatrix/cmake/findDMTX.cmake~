# Get hint from environment variable (if any)
if(NOT DMTX_ROOT_DIR AND DEFINED ENV{DMTX_ROOT_DIR})
  set(DMTX_ROOT_DIR "$ENV{DMTX_ROOT_DIR}" CACHE PATH
      "DMTX base directory location (optional, used for nonstandard installation paths)")
endif()

# Search path for nonstandard locations
if(DMTX_ROOT_DIR)
  set(DMTX_INCLUDE_PATH PATHS "${DMTX_ROOT_DIR}/include" NO_DEFAULT_PATH)
  set(DMTX_LIBRARY_PATH PATHS "${DMTX_ROOT_DIR}/lib"     NO_DEFAULT_PATH)
endif()

# Find headers and libraries
find_path(DMTX_INCLUDE_DIR NAMES dmtx.h PATH_SUFFIXES "dmtx" ${DMTX_INCLUDE_PATH})
find_library(DMTX_LIBRARY  NAMES libdmtx dmtx PATH_SUFFIXES "dmtx" ${DMTX_LIBRARY_PATH})

mark_as_advanced(DMTX_INCLUDE_DIR
                 DMTX_LIBRARY)

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DMTX DEFAULT_MSG DMTX_LIBRARY
                                                      DMTX_INCLUDE_DIR)

if(DMTX_FOUND)
  set(DMTX_INCLUDE_DIRS ${DMTX_INCLUDE_DIR})
  set(DMTX_LIBRARIES ${DMTX_LIBRARY})
endif()
