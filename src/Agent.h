/*
 * Agent.h
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

#ifndef RVO2_AGENT_H_
#define RVO2_AGENT_H_

/**
 * @file  Agent.h
 * @brief Declares the Agent class.
 */
#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "Goal.h"
#include "Line.h"
#include "Obstacle.h"
#include "RVO2Simulator.h"
#include "Vector2.h"

namespace RVO2 {
class KdTree;
class Obstacle;

class Candidate {
 public:
  Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) {}

  Vector2 position_;
  int velocityObstacle1_;
  int velocityObstacle2_;
};

/**
 * @brief A hybrid reciprocal velocity obstacle.
 */
class VelocityObstacle {
 public:
  /**
   * @brief The position of the apex of the hybrid reciprocal velocity
   *        obstacle.
   */
  Vector2 apex_;

  /**
   * @brief The direction of the first side of the hybrid reciprocal velocity
   *        obstacle.
   */
  Vector2 side1_;

  /**
   * @brief The direction of the second side of the hybrid reciprocal velocity
   *        obstacle.
   */
  Vector2 side2_;
};

/**
 * @brief Defines an agent in the simulation.
 */
class Agent {
 private:
  /**
   * @brief Constructs an agent instance.
   */
  explicit Agent();

  /**
   * @brief Destroys this agent instance.
   */
  ~Agent();

  /* Not implemented. */
  explicit Agent(const Agent &other);

  /* Not implemented. */
  Agent &operator=(const Agent &other);

  /**
   * @brief     Constructor.
   * @param[in] simulator The simulation.
   */
  explicit Agent(RVO2Simulator *simulator);

  /**
   * @brief     Constructor.
   * @param[in] simulator The simulation.
   * @param[in] position  The starting position of this agent.
   * @param[in] goalNo    The goal number of this agent.
   */
  Agent(RVO2Simulator *simulator, const Vector2 &position, std::size_t goalNo);

  /**
   * @brief     Constructor.
   * @param[in] simulator         The simulation.
   * @param[in] position          The starting position of this agent.
   * @param[in] goalNo            The goal number of this agent.
   * @param[in] neighborDist      The maximum neighbor distance of this agent.
   * @param[in] maxNeighbors      The maximum neighbor count of this agent.
   * @param[in] radius            The radius of this agent.
   * @param[in] goalRadius        The goal radius of this agent.
   * @param[in] prefSpeed         The preferred speed of this agent.
   * @param[in] maxSpeed          The maximum speed of this agent.
   * @param[in] uncertaintyOffset The uncertainty offset of this agent.
   * @param[in] maxAccel          The maximum acceleration of this agent.
   * @param[in] velocity          The initial velocity of this agent.
   * @param[in] orientation       The initial orientation (in radians) of this
   * agent.
   */
  Agent(RVO2Simulator *simulator, const Vector2 &position, std::size_t goalNo,
        float neighborDist, std::size_t maxNeighbors, float radius,
        const Vector2 &velocity, float maxAccel, float goalRadius,
        float prefSpeed, float maxSpeed, float orientation,
        float uncertaintyOffset);

  /**
   * @brief     Computes the neighbors of this agent.
   * @param[in] kdTree A pointer to the k-D trees for agents and static
   *                   obstacles in the simulation.
   */
  void computeNeighbors(const KdTree *kdTree);

  /**
   * @brief     Computes the new velocity of this agent.
   * @param[in] timeStep The time step of the simulation.
   */
  void computeNewVelocity(float timeStep);

  /**
   * @brief Computes the new velocity of this agent.
   */
  void computeNewVelocity();

  /**
   * @brief Computes the preferred velocity of this agent.
   */
  void computePreferredVelocity();

  /**
   * @brief          Inserts an agent neighbor into the set of neighbors of this
   *                 agent.
   * @param[in]      agent   A pointer to the agent to be inserted.
   * @param[in, out] rangeSq The squared range around this agent.
   */
  void insertAgentNeighbor(const Agent *agent,
                           float &rangeSq); /* NOLINT(runtime/references) */

  /**
   * @brief          Inserts a static obstacle neighbor into the set of
   *                 neighbors of this agent.
   * @param[in]      obstacle The number of the static obstacle to be inserted.
   * @param[in, out] rangeSq  The squared range around this agent.
   */
  void insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq);

  /**
   * @brief     Updates the two-dimensional position and two-dimensional
   *            velocity of this agent.
   * @param[in] timeStep The time step of the simulation.
   */
  void update(float timeStep);

  RVO2Simulator *const simulator_;
  std::size_t id_;
  std::vector<std::pair<float, const Agent *> > agentNeighbors_;
  std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;
  std::multimap<float, Candidate> candidates_;
  std::vector<VelocityObstacle> velocityObstacles_;
  std::vector<Line> orcaLines_;
  Vector2 newVelocity_;
  Vector2 position_;
  float orientation_;
  Vector2 prefVelocity_;
  Vector2 velocity_;
  float neighborDist_;
  std::size_t maxNeighbors_;
  float maxAccel_;
  float maxSpeed_;
  float prefSpeed_;
  float radius_;
  float goalRadius_;
  std::size_t goalNo_;
  float timeHorizon_;
  float timeHorizonObst_;
  float uncertaintyOffset_;

  bool reachedGoal_;

  friend class KdTree;
  friend class RVO2Simulator;
};
} /* namespace RVO2 */

#endif /* RVO2_AGENT_H_ */
