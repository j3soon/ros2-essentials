# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# generate version header file from package.xml
file(READ "${PROJECT_SOURCE_DIR}/package.xml" PACKAGE_MANIFEST)
string(REGEX MATCH "<version>([0-9]).[0-9].[0-9]</version>" _ ${PACKAGE_MANIFEST})
set(${PROJECT_NAME}_VERSION_MAJOR ${CMAKE_MATCH_1})
string(REGEX MATCH "<version>[0-9].([0-9]).[0-9]</version>" _ ${PACKAGE_MANIFEST})
set(${PROJECT_NAME}_VERSION_MINOR ${CMAKE_MATCH_1})
string(REGEX MATCH "<version>[0-9].[0-9].([0-9])</version>" _ ${PACKAGE_MANIFEST})
set(${PROJECT_NAME}_VERSION_PATCH ${CMAKE_MATCH_1})
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})
if(NOT ${PROJECT_NAME}_VERSION MATCHES "[0-9].[0-9].[0-9]")
  message(FATAL_ERROR "Could not parse version from package.xml or version is not formatted properly (should be MAJOR.MINOR.PATCH).")
endif()

set(PATH_TO_VERSION_TEMPLATE ${PROJECT_SOURCE_DIR}/cmake/version.hpp.in)
set(PATH_TO_VERSION_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/version.hpp)
if(NOT EXISTS ${PATH_TO_VERSION_TEMPLATE})
  message(FATAL_ERROR "Could not locate version template file at '${PATH_TO_VERSION_TEMPLATE}'.")
endif()

configure_file(
  ${PATH_TO_VERSION_TEMPLATE}
  ${PATH_TO_VERSION_OUTPUT}
  @ONLY
)
