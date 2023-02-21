# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# examples/CMakeLists.txt
# RVO2 Library
#
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Please send all bug reports to <geom@cs.unc.edu>.
#
# The authors may be contacted via:
#
# Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
# Dept. of Computer Science
# 201 S. Columbia St.
# Frederick P. Brooks, Jr. Computer Science Bldg.
# Chapel Hill, N.C. 27599-3175
# United States of America
#
# <https://gamma.cs.unc.edu/RVO2/>
#

if(BUILD_TESTING)
  option(OUTPUT_TIME_AND_POSITIONS "Output time and positions" ON)

  if(OUTPUT_TIME_AND_POSITIONS)
    set(RVO2_EXAMPLES_COMPILE_DEFINITIONS RVO2_OUTPUT_TIME_AND_POSITIONS=1)
  else()
    set(RVO2_EXAMPLES_COMPILE_DEFINITIONS RVO2_OUTPUT_TIME_AND_POSITIONS=0)
  endif()

  option(SEED_RANDOM_NUMBER_GENERATOR "Seed random number generator" ON)

  if(SEED_RANDOM_NUMBER_GENERATOR)
    list(APPEND RVO2_EXAMPLES_COMPILE_DEFINITIONS
      RVO2_SEED_RANDOM_NUMBER_GENERATOR=1)
  else()
    list(APPEND RVO2_EXAMPLES_COMPILE_DEFINITIONS
      RVO2_SEED_RANDOM_NUMBER_GENERATOR=0)
  endif()

  if(ENABLE_OPENMP AND OpenMP_FOUND)
    set(RVO2_EXAMPLES_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    set(RVO2_EXAMPLES_LINK_FLAGS "${OpenMP_CXX_FLAGS}")
  else()
    set(RVO2_EXAMPLES_COMPILE_OPTIONS)
    set(RVO2_EXAMPLES_LINK_FLAGS)
  endif()

  if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
      AND RVO2_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
    set(RVO2_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION ON)
  else()
    set(RVO2_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION OFF)
  endif()

  add_executable(Blocks Blocks.cc)
  target_compile_definitions(Blocks PRIVATE
    ${RVO2_EXAMPLES_COMPILE_DEFINITIONS})
  target_link_libraries(Blocks PRIVATE ${RVO2_LIBRARY})
  target_compile_options(Blocks PRIVATE ${RVO2_EXAMPLES_COMPILE_OPTIONS})
  set_target_properties(Blocks PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ${RVO2_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION}
    LINK_FLAGS "${RVO2_EXAMPLES_LINK_FLAGS}")
  add_test(NAME Blocks COMMAND Blocks)
  set_tests_properties(Blocks PROPERTIES
    LABELS medium
    TIMEOUT 60)

  add_executable(Circle Circle.cc)
  target_compile_definitions(Circle PRIVATE
    ${RVO2_EXAMPLES_COMPILE_DEFINITIONS})
  target_link_libraries(Circle PRIVATE ${RVO2_LIBRARY})
  target_compile_options(Circle PRIVATE ${RVO2_EXAMPLES_COMPILE_OPTIONS})
  set_target_properties(Circle PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ${RVO2_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION}
    LINK_FLAGS "${RVO2_EXAMPLES_LINK_FLAGS}")
  add_test(NAME Circle COMMAND Circle)
  set_tests_properties(Circle PROPERTIES
    LABELS medium
    TIMEOUT 60)

  add_executable(Roadmap Roadmap.cc)
  target_compile_definitions(Roadmap PRIVATE
    ${RVO2_EXAMPLES_COMPILE_DEFINITIONS})
  target_link_libraries(Roadmap PRIVATE ${RVO2_LIBRARY})
  target_compile_options(Roadmap PRIVATE ${RVO2_EXAMPLES_COMPILE_OPTIONS})
  set_target_properties(Roadmap PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ${RVO2_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION}
    LINK_FLAGS "${RVO2_EXAMPLES_LINK_FLAGS}")
  add_test(NAME Roadmap COMMAND Roadmap)
  set_tests_properties(Roadmap PROPERTIES
    LABELS medium
    TIMEOUT 60)
endif()
