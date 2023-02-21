/*
 * Circle.cc
 * RVO2 Library
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

/*
 * @file  Circle.cc
 * @brief Example file showing a demo with 250 agents initially positioned
 *        evenly distributed on a circle attempting to move to the antipodal
 *        position on the circle.
 */

#ifndef RVO2_OUTPUT_TIME_AND_POSITIONS
#define RVO2_OUTPUT_TIME_AND_POSITIONS 1
#endif /* RVO2_OUTPUT_TIME_AND_POSITIONS */

#include <cmath>
#include <vector>

#if RVO2_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif /* RVO2_OUTPUT_TIME_AND_POSITIONS */

#if _OPENMP
#include <omp.h>
#endif /* _OPENMP */

#include "RVO2.h"

namespace {
const float RVO2_TWO_PI = 6.28318530717958647692F;

void setupScenario(
    RVO2::RVO2Simulator *simulator,
    std::vector<RVO2::Vector2> &goals) { /* NOLINT(runtime/references) */
  /* Specify the global time step of the simulation. */
  simulator->setTimeStep(0.25F);

  /* Specify the default parameters for agents that are subsequently added. */
  simulator->setAgentDefaults(15.0F, 10U, 10.0F, 10.0F, 1.5F, 2.0F);

  /* Add agents, specifying their start position, and store their goals on the
   * opposite side of the environment. */
  for (std::size_t i = 0U; i < 250U; ++i) {
    simulator->addAgent(
        200.0F *
        RVO2::Vector2(std::cos(static_cast<float>(i) * RVO2_TWO_PI * 0.004F),
                      std::sin(static_cast<float>(i) * RVO2_TWO_PI * 0.004F)));
    goals.push_back(-simulator->getAgentPosition(i));
  }
}

#if RVO2_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO2::RVO2Simulator *simulator) {
  /* Output the current global time. */
  std::cout << simulator->getGlobalTime();

  /* Output the current position of all the agents. */
  for (std::size_t i = 0U; i < simulator->getNumAgents(); ++i) {
    std::cout << " " << simulator->getAgentPosition(i);
  }

  std::cout << std::endl;
}
#endif /* RVO2_OUTPUT_TIME_AND_POSITIONS */

void setPreferredVelocities(RVO2::RVO2Simulator *simulator,
                            const std::vector<RVO2::Vector2> &goals) {
  /* Set the preferred velocity to be a vector of unit magnitude (speed) in the
   * direction of the goal. */
#ifdef _OPENMP
#pragma omp parallel for
#endif /* _OPENMP */
  for (int i = 0; i < static_cast<int>(simulator->getNumAgents()); ++i) {
    RVO2::Vector2 goalVector = goals[i] - simulator->getAgentPosition(i);

    if (RVO2::absSq(goalVector) > 1.0F) {
      goalVector = RVO2::normalize(goalVector);
    }

    simulator->setAgentPrefVelocity(i, goalVector);
  }
}

bool reachedGoal(RVO2::RVO2Simulator *simulator,
                 const std::vector<RVO2::Vector2> &goals) {
  /* Check if all agents have reached their goals. */
  for (std::size_t i = 0U; i < simulator->getNumAgents(); ++i) {
    if (RVO2::absSq(simulator->getAgentPosition(i) - goals[i]) >
        simulator->getAgentRadius(i) * simulator->getAgentRadius(i)) {
      return false;
    }
  }

  return true;
}
} /* namespace */

int main() {
  /* Store the goals of the agents. */
  std::vector<RVO2::Vector2> goals;

  /* Create a new simulator instance. */
  RVO2::RVO2Simulator *simulator = new RVO2::RVO2Simulator();

  /* Set up the scenario. */
  setupScenario(simulator, goals);

  /* Perform and manipulate the simulation. */
  do {
#if RVO2_OUTPUT_TIME_AND_POSITIONS
    updateVisualization(simulator);
#endif /* RVO2_OUTPUT_TIME_AND_POSITIONS */
    setPreferredVelocities(simulator, goals);
    simulator->doStep();
  } while (!reachedGoal(simulator, goals));

  delete simulator;

  return 0;
}
