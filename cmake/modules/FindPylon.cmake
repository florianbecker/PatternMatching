#/*
# * Copyright (c) 2020 Florian Becker <fb@vxapps.com> (VX APPS).
# * All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * 1. Redistributions of source code must retain the above copyright notice, this
# *    list of conditions and the following disclaimer.
# *
# * 2. Redistributions in binary form must reproduce the above copyright notice,
# *    this list of conditions and the following disclaimer in the documentation
# *    and/or other materials provided with the distribution.
# *
# * 3. Neither the name of the copyright holder nor the names of its
# *    contributors may be used to endorse or promote products derived from
# *    this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

if(APPLE)
  find_path(PYLON_INCLUDE_DIR pylon/PylonIncludes.h)
  find_path(PYLON_BASE_INCLUDE_DIR
    NAMES Base/GCTypes.h
    HINTS ${PYLON_INCLUDE_DIR}/Headers
    PATH_SUFFIXES GenICam)

  find_library(PYLON_LIBRARIES pylon)

  find_program(PYLON_CONFIG pylon-config
               HINTS ${PYLON_INCLUDE_DIR}
               PATH_SUFFIXES Versions/A/Resources/Tools)

  if(PYLON_INCLUDE_DIR)
    list(APPEND PYLON_INCLUDE_DIRS ${PYLON_INCLUDE_DIR})
  endif()
  if(PYLON_BASE_INCLUDE_DIR)
    list(APPEND PYLON_INCLUDE_DIRS ${PYLON_BASE_INCLUDE_DIR})
  endif()

  if(PYLON_CONFIG)
    execute_process(COMMAND ${PYLON_CONFIG} "--version"
                    OUTPUT_VARIABLE PYLON_VERSION_TMP
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE "-n" "" PYLON_VERSION ${PYLON_VERSION_TMP})
  endif()

  if(PYLON_INCLUDE_DIRS AND PYLON_LIBRARIES)
    set(PYLON_FOUND TRUE)
  endif()

  mark_as_advanced(PYLON_BASE_INCLUDE_DIR)
elseif(MSVC)
  find_path(PYLON_INCLUDE_DIR pylon/PylonIncludes.h
    PATHS "$ENV{PYLON_HOME}/include"
        "C:/Program Files/Basler/pylon 5/Development/include")

  set(PYLON_LIB_SEARCH_PATH "$ENV{PYLON_HOME}/lib/x64")

  if(CMAKE_CL_64)
    list(APPEND PYLON_LIB_SEARCH_PATH "C:/Program Files/Basler/pylon 5/Development/lib/x64")
  else()
    list(APPEND PYLON_LIB_SEARCH_PATH "C:/Program Files/Basler/pylon 5/Development/lib/Win32")
  endif()

  find_library(PYLON_BASE_LIBRARY
    NAMES PylonBase_v5_1.lib PylonBase_MD_VC120_v5_0.lib
    PATHS ${PYLON_LIB_SEARCH_PATH})
  find_library(PYLON_GCBASE_LIBRARY
    NAMES GCBase_MD_VC141_v3_1_Basler_pylon_v5_1.lib GCBase_MD_VC120_v3_0_Basler_pylon_v5_0.lib
    PATHS ${PYLON_LIB_SEARCH_PATH})
  find_library(PYLON_GENAPI_LIBRARY
    NAMES GenApi_MD_VC141_v3_1_Basler_pylon_v5_1.lib GenApi_MD_VC120_v3_0_Basler_pylon_v5_0.lib
    PATHS ${PYLON_LIB_SEARCH_PATH})
  find_library(PYLON_UTILITY_LIBRARY
    NAMES PylonUtility_v5_1.lib PylonUtility_MD_VC120_v5_0.lib
    PATHS ${PYLON_LIB_SEARCH_PATH})

  if(PYLON_INCLUDE_DIR)
    list(APPEND PYLON_INCLUDE_DIRS ${PYLON_INCLUDE_DIR})
  endif()
  if(PYLON_BASE_LIBRARY AND PYLON_GCBASE_LIBRARY)
    list(APPEND PYLON_LIBRARIES ${PYLON_BASE_LIBRARY})
    list(APPEND PYLON_LIBRARIES ${PYLON_GCBASE_LIBRARY})
    list(APPEND PYLON_LIBRARIES ${PYLON_GENAPI_LIBRARY})
    list(APPEND PYLON_LIBRARIES ${PYLON_UTILITY_LIBRARY})
  endif()

  if(PYLON_INCLUDE_DIRS AND PYLON_LIBRARIES)
    vp_parse_header("${PYLON_INCLUDE_DIR}/pylon/PylonVersionNumber.h" PYLON_VERSION_LINES PYLON_VERSION_MAJOR PYLON_VERSION_MINOR PYLON_VERSION_SUBMINOR)
    set(PYLON_VERSION "${PYLON_VERSION_MAJOR}.${PYLON_VERSION_MINOR}.${PYLON_VERSION_SUBMINOR}")
    set(PYLON_FOUND TRUE)
  endif()

  mark_as_advanced(
    PYLON_BASE_LIBRARY
    PYLON_GCBASE_LIBRARY
    PYLON_GENAPI_LIBRARY
    PYLON_UTILITY_LIBRARY
  )

elseif(UNIX)
  set(PYLON_ROOT_SEARCH_PATH /opt/pylon)
  # For more possible versions, just add more paths below.
  list(APPEND PYLON_ROOT_SEARCH_PATH "/opt/pylon5")

  find_program(PYLON_CONFIG pylon-config
               PATHS ${PYLON_ROOT}
               PATHS $ENV{PYLON_ROOT}
               PATHS ${PYLON_ROOT_SEARCH_PATH}
               PATH_SUFFIXES bin)

  if(PYLON_CONFIG)
    execute_process(COMMAND ${PYLON_CONFIG} "--version"
                    OUTPUT_VARIABLE PYLON_VERSION
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${PYLON_CONFIG} "--libs" "--libs-rpath"
                    OUTPUT_VARIABLE PYLON_LIBRARIES
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${PYLON_CONFIG} "--cflags-only-I"
                    OUTPUT_VARIABLE PYLON_INC_TMP
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE "-I" "" PYLON_INCLUDE_DIRS ${PYLON_INC_TMP})

    set(PYLON_FOUND TRUE)
  endif()
endif()

mark_as_advanced(
  PYLON_INCLUDE_DIR
  PYLON_INCLUDE_DIRS
  PYLON_LIBRARIES
  PYLON_CONFIG
)
