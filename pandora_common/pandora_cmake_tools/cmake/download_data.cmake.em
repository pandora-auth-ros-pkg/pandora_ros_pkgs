# Original idea: https://github.com/ros/catkin/pull/633

#
# Download a file containing data from a URL.
#
# It is commonly used to download larger data files which should not be
# stored in the repository.
#
# .. note:: The target will be registered as a dependency
#   of the "download_extra_data" target.
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string
#
# :param DESTINATION: the directory where the file is downloaded to
#   (default: ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
# :type DESTINATION: string
# :param FILENAME: the filename of the downloaded file
#   (default: the basename of the url)
# :type FILENAME: string
# :param MD5: the expected md5 hash to compare against
#   (default: empty, skipping the check)
# :type MD5: string
#
# :param EXCLUDE_FROM_ALL: exclude this download from the 'all' build target
# :type EXCLUDE_FROM_ALL: optional
# :param OPTIONAL: don't require the file to be available for the build to succeed
# :type OPTIONAL: optional
#
function(download_data target url)
  cmake_parse_arguments(ARG "EXCLUDE_FROM_ALL;OPTIONAL"
    "DESTINATION;FILENAME;MD5;" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "download_data() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DESTINATION)
    set(ARG_DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}")
  endif()

  if(NOT ARG_FILENAME)
    get_filename_component(ARG_FILENAME ${url} NAME)
  endif()

  set(optional "")
  if(ARG_OPTIONAL)
      set(optional "--ignore-error")
  endif()

  set(output "${ARG_DESTINATION}/${ARG_FILENAME}")

@[if DEVELSPACE]@
  # bin in develspace
  set(DOWNLOAD_CHECKMD5_SCRIPT "@(PROJECT_SOURCE_DIR)/scripts/download_checkmd5.py")
@[else]@
  # bin in installspace
  set(DOWNLOAD_CHECKMD5_SCRIPT "@(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_SHARE_DESTINATION)/scripts/download_checkmd5.py")
@[end if]@

  # With this, the command is always called, even when the output is up to date.
  # this is because we want to check the md5 sum if it's given, and redownload
  # the target if the md5 sum does not match.
  add_custom_target(${target} ALL
    COMMAND ${DOWNLOAD_CHECKMD5_SCRIPT} ${url} ${output} ${ARG_MD5} ${optional}
    VERBATIM)

  if(TARGET download_extra_data)
    add_dependencies(download_extra_data ${target})
  endif()

  if(ARG_EXCLUDE_FROM_ALL)
    set_target_properties(${target} PROPERTIES
      EXCLUDE_FROM_ALL true)
  endif()

endfunction()

#
# Download multiple files using a 'extra_files.py' for convinience.
#
# it is commonly used to download larger data files which should not be
# stored in the repository, and which are not required as part of the build
#
# .. note:: the target will be registered as a dependency
#   of the "download_extra_data" target
#
# :param target: the target name
# :type target: string
# :param extra_files: paths to 'extra_files.py'
# :type url: string
#
# :param EXCLUDE_FROM_ALL: exclude this download from the 'all' build target
# :type EXCLUDE_FROM_ALL: optional
#
function(download_extra_data target)
  cmake_parse_arguments(ARG "EXCLUDE_FROM_ALL" "" "EXTRA_FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "download_multiple_data() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  set(FINAL_EXTRA_FILES "")
  foreach(extra_file ${ARG_EXTRA_FILES})
    list(APPEND FINAL_EXTRA_FILES "${PROJECT_SOURCE_DIR}/${extra_file}")
  endforeach()

@[if DEVELSPACE]@
  # bin in develspace
  set(PARSE_EXTRA_FILES_SCRIPT "@(PROJECT_SOURCE_DIR)/scripts/parse_extra_files.py")
@[else]@
  # bin in installspace
  set(PARSE_EXTRA_FILES_SCRIPT "@(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_SHARE_DESTINATION)/scripts/parse_extra_files.py")
@[end if]@

  # With this, the command is always called, even when the output is up to date.
  # this is because we want to check the md5 sum if it's given, and redownload
  # the target if the md5 sum does not match.
  add_custom_target(${target} ALL
    COMMAND ${PARSE_EXTRA_FILES_SCRIPT} "--project-source-dir" ${PROJECT_SOURCE_DIR}
      "--project-devel-dir" ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
      ${FINAL_EXTRA_FILES}
    VERBATIM)

  if(TARGET download_extra_data)
    add_dependencies(download_extra_data ${target})
  endif()

  if(ARG_EXCLUDE_FROM_ALL)
    set_target_properties(${target} PROPERTIES
      EXCLUDE_FROM_ALL true)
  endif()

endfunction()

if(NOT TARGET download_extra_data)
  add_custom_target(download_extra_data)
endif()
