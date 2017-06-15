/*
 * Agent.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace RVO
{
    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    internal abstract class Agent : SuperAgent
    {
        internal IList<KeyValuePair<float, Agent>> agentNeighbors_ = new List<KeyValuePair<float, Agent>>();
        internal IList<KeyValuePair<float, Obstacle>> obstacleNeighbors_ = new List<KeyValuePair<float, Obstacle>>();
        internal IList<Line> orcaLines_ = new List<Line>();
        internal Vector2 prefVelocity_;
        internal int id_ = 0;
        internal int maxNeighbors_ = 0;
        internal float maxSpeed_ = 0.0f;
        internal float neighborDist_ = 0.0f;
        internal float timeHorizon_ = 0.0f;
        internal float timeHorizonObst_ = 0.0f;
        internal Vector2 newVelocity_;
        public IList<Vector2> velocityBuffer_ = new List<Vector2>();
        internal Vector2 acceleration_;
        internal Vector2 goal_;
        internal float cpt_dist = 0;
        internal int cpt_time = 0;
        public bool considerGroups_;
        public GroupAgent groupBelongingTo_;
        public bool follows_;
        public Agent leader_;
        float c_ = 1.3f ;
		float tau_ = 0.2f;
		float gamma_ = 0.5f;
        public Pair<float, float> terrain_max_speeds_;
        internal float velocityMed =0;

        public Agent(RVOSimulator sim) : base(sim) { }

        /**
         * <summary>Computes the neighbors of this agent.</summary>
         */
        internal void computeNeighbors()
        {
            obstacleNeighbors_.Clear();
            float rangeSq = RVOMath.sqr(timeHorizonObst_ * maxSpeed_ + radius_);
            sim_.kdTree_.computeObstacleNeighbors(this, rangeSq);
            
            agentNeighbors_.Clear();

            if (maxNeighbors_ > 0)
            {
                rangeSq =RVOMath.sqr(neighborDist_/1.5f) + RVOMath.absSq(velocity_);// RVOMath.sqr(neighborDist_);
                sim_.kdTree_.computeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>Computes the new velocity of this agent.</summary>
         */
        public abstract void computeNewVelocity();
      
        /**
         * <summary>Computes the interaction between this agent and the one in parameter.</summary>
         */
        public abstract void interactWith(SuperAgent agent);

        /**
         * <summary>Computes the interaction between this agent and the obstacles in parameter.</summary>
         */
        public abstract void interactWith(Obstacle obstacle);

        /**
        * <summary>Computes the interaction between this agent and all the others agents.</summary>
        */
        public void interactWithAgents()
        {
            IList<GroupAgent> groups = new List<GroupAgent>();
            for (int i = 0; i < agentNeighbors_.Count; ++i)
            {
                Agent neighbor = agentNeighbors_[i].Value;
                if (considerGroups_)
                {
                    bool present = false;
                    GroupAgent agentGroup = neighbor.groupBelongingTo_;
                    // If the agent does not belong to any group or if it belongs to the same group, interact with it
                    if (agentGroup == null || agentGroup == groupBelongingTo_)
                        interactWith(neighbor);
                    // Otherwise, add its group to the list of the groups to interact with
                    else
                    {

                        foreach (GroupAgent g in groups)
                        {
                            if (agentGroup == g)
                            {
                                present = true;
                            }
                        }


                        // If the group has not been selected yet, add it to the group list
                        if (!present)
                            groups.Add(agentGroup);
                    }
                }
                else
                {
                    interactWith(neighbor);
                }
            }
            // Interact with each group of the list
            for (int i = 0; i < groups.Count; ++i)
            {
                GroupAgent group = groups[i];
                // If the group's representation succeded, interact with it
                SuperAgent groupRepresentation = group.representGroup(this);
                if (groupRepresentation.sim_ != null)
                {
                    interactWith(groupRepresentation);
                }
                // Otherwise, interact with each agent of the group
                else
                {
                    IList<Agent> agentsOfTheGroup = group.getAgents();
                    foreach (Agent a in agentsOfTheGroup)
                    {
                        interactWith(a);
                    }

                }
            }
        }


        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertAgentNeighbor(Agent agent, ref float rangeSq)
        {
            if (this != agent)
            {
                float distSq = RVOMath.absSq(position_ - agent.position_);

                if (distSq < rangeSq)
                {
                    if (agentNeighbors_.Count < maxNeighbors_)
                    {
                        agentNeighbors_.Add(new KeyValuePair<float, Agent>(distSq, agent));

                    }

                    int i = agentNeighbors_.Count - 1;

                    while (i != 0 && distSq < agentNeighbors_[i - 1].Key)
                    {
                        agentNeighbors_[i] = agentNeighbors_[i - 1];
                        --i;
                    }

                    agentNeighbors_[i] = new KeyValuePair<float, Agent>(distSq, agent);

                    if (agentNeighbors_.Count == maxNeighbors_)
                    {
                        rangeSq = agentNeighbors_[agentNeighbors_.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertObstacleNeighbor(Obstacle obstacle, float rangeSq)
        {
            Obstacle nextObstacle = obstacle.next_;

            float distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position_);

            if (distSq < rangeSq)
            {
                obstacleNeighbors_.Add(new KeyValuePair<float, Obstacle>(distSq, obstacle));

                int i = obstacleNeighbors_.Count - 1;

                while (i != 0 && distSq < obstacleNeighbors_[i - 1].Key)
                {
                    obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                    --i;
                }
                obstacleNeighbors_[i] = new KeyValuePair<float, Obstacle>(distSq, obstacle);
            }
        }

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void update()
        {
            if (sim_.pas < 1)
            {
                velocityMed = Vector2.abs(velocity_);
            }
            else
            {
                velocityMed += Vector2.abs(velocity_);
                velocityMed /= 2;
            }
            velocityBuffer_.Add(velocity_);
            if (!Double.IsNaN(newVelocity_.x_) && !Double.IsNaN(newVelocity_.y_))
            {

                velocity_ = newVelocity_;
                position_ += velocity_ * sim_.timeStep_;
                cpt_dist += Vector2.abs(velocity_) * sim_.getTimeStep();
                cpt_time++;
            }
            else
            {
                newVelocity_ = new Vector2(0, 0);
                velocity_ = newVelocity_;
                position_ += velocity_ * sim_.timeStep_;
                cpt_dist += Vector2.abs(velocity_) * sim_.getTimeStep();
                cpt_time++;
            }
        }

        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        public bool linearProgram1(IList<Line> lines, int lineNo, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            float dotProduct = lines[lineNo].point * lines[lineNo].direction;
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = RVOMath.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                float numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (RVOMath.fabs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (optVelocity * lines[lineNo].direction > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        public int linearProgram2(IList<Line> lines, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        public void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, float radius, ref Vector2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (RVOMath.fabs(determinant) <= RVOMath.RVO_EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (lines[i].direction * lines[j].direction > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5f *(lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    Vector2 tempResult = result;
                    if (linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, ref result) < projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }

        /**
        * <summary>Apply the model to the Agents</summary>
        */
        public void applymodel()
        {

            computeNewVelocity();
            acceleration_ = (newVelocity_ - velocity_) / sim_.getTimeStep();
            if (follows_)
            {
                selectLeader();
                float acc_follow = followingBehavior();
                applyFollowingBehavior(acc_follow);
            }
        }

        /**
        * <summary>Apply the behaviour of following to the agent</summary>
        */
        void applyFollowingBehavior(float following_acc)
        {
            float alpha = Vector2.angle(velocity_);
            // If the result lowers the tangential component of the acceleration,
            // apply this tangential acceleration to compute the new velocity
            Vector2 local_acceleration = Vector2.rotation(acceleration_, -alpha);
            if (following_acc < local_acceleration.x())
            {   
                Vector2 local_new_acceleration = new Vector2(following_acc, local_acceleration.y());
                acceleration_ = Vector2.rotation(local_new_acceleration, alpha);                
                newVelocity_ = velocity_ + acceleration_ * sim_.getTimeStep();
            }
        }

        /**
        * <summary>Following Behaviour of an agent</summary>
        */
        float followingBehavior()
        {

            // Compute pedestrian angle for local referential
            float alpha = Vector2.angle(velocity_);
            // If there is a leader to follow
            if (leader_ != null)
            {

                if (velocityBuffer_.Count > 0 && leader_.velocityBuffer_.Count > 0)
                {
                    // Compute relative position
                    Vector2 relative_pos = leader_.position_ - position_;
                    Vector2 local_relative_pos = Vector2.rotation(relative_pos, -alpha);

                    // Compute related velocity with tau delay
                    Vector2 delayed_relative_vel;
                    int element = (int)Math.Round((velocityBuffer_.Count - 1 )- tau_ / sim_.getTimeStep());
                    element = Math.Max(0, element);
                    delayed_relative_vel = leader_.velocityBuffer_[element] - velocityBuffer_[element];
                    Vector2 delayed_local_relative_vel = Vector2.rotation(delayed_relative_vel, -alpha);

                    // Apply following model
                    return lemercier(delayed_local_relative_vel.x(), local_relative_pos.x());
                }
                else
                {
                    return Vector2.rotation(acceleration_,-alpha).x();
                }
            }
            else
            {
                return Vector2.rotation(acceleration_,-alpha).x();
            }
        }

        float lemercier(float dv, float dx)
        {
            float a = c_ * dv / (float)Math.Pow(dx, gamma_);
            return a;
        }

        /**
        * <summary>Select the Leader of the agent</summary>
        */
        void selectLeader()
        {
            leader_ = null;
            for (int neighbor_id = 0; neighbor_id < agentNeighbors_.Count; ++neighbor_id)
            {
                Agent neighbor = agentNeighbors_[neighbor_id].Value;
                Vector2 relative_pos = neighbor.position_ - position_;
                float alpha = Vector2.angle(velocity_);
                Vector2 local_relative_pos = Vector2.rotation(relative_pos, -alpha);
                Vector2 local_velocity = Vector2.rotation(neighbor.velocity_, -alpha);
                float alpha_v = Vector2.angle(local_velocity);
               
                if (local_relative_pos.x() > 0
                    && local_relative_pos.x() < 1.5
                    && Math.Abs(local_relative_pos.y()) < this.radius_ + neighbor.radius_
                    && Math.Abs(alpha_v) < Math.PI / 6
                    && local_velocity.x() >= 0
                    )
                {
                    if (leader_ == null)
                        leader_ = neighbor;
                    else
                    {
                        Vector2 leader_relative_pos = leader_.position_ - position_;
                        Vector2 leader_local_relative_pos = Vector2.rotation(leader_relative_pos, -alpha);
                        if (leader_local_relative_pos.x() > local_relative_pos.x())
                        {
                            leader_ = neighbor;
                        }
                    }
                }
            }
        }
    }
}
