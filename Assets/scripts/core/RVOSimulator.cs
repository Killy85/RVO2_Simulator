/*
*  RVOSimulator.cs
*  RVO2 Library.
*  
*  
*  Copyright (C) 2008-10 University of North Carolina at Chapel Hill.
*  All rights reserved.
*  
*  Permission to use, copy, modify, and distribute this software and its
*  documentation for educational, research, and non-profit purposes, without
*  fee, and without a written agreement is hereby granted, provided that the
*  above copyright notice, this paragraph, and the following four paragraphs
*  appear in all copies.
*  
*  Permission to incorporate this software into commercial products may be
*  obtained by contacting the University of North Carolina at Chapel Hill.
*  
*  This software program and documentation are copyrighted by the University of
*  North Carolina at Chapel Hill. The software program and documentation are
*  supplied "as is", without any accompanying services from the University of
*  North Carolina at Chapel Hill or the authors. The University of North
*  Carolina at Chapel Hill and the authors do not warrant that the operation of
*  the program will be uninterrupted or error-free. The end-user understands
*  that the program was developed for research purposes and is advised not to
*  rely exclusively on the program for any reason.
*  
*  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR ITS
*  EMPLOYEES OR THE AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
*  SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS,
*  ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE
*  UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED
*  OF THE POSSIBILITY OF SUCH DAMAGE.
*  
*  THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
*  DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
*  STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS
*  ON AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND
*  THE AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
*  ENHANCEMENTS, OR MODIFICATIONS.
*  
*  Please send all BUG REPORTS to:
*  
*  geom@cs.unc.edu
*  
*  The authors may be contacted via:
*  
*  Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, and
*  Dinesh Manocha
*  Dept. of Computer Science
*  Frederick P. Brooks Jr. Computer Science Bldg.
*  3175 University of N.C.
*  Chapel Hill, N.C. 27599-3175
*  United States of America
*  
*  http://gamma.cs.unc.edu/RVO2/
*  
*  Modified by Samuel Lemercier
*/


using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using UnityEngine;

namespace RVO
{

    public class RVOSimulator
    {
        private class Worker
        {
            private ManualResetEvent doneEvent_;
            internal int end_;
            private int start_;
            private RVOSimulator simulator_;
            private bool looped;

            /**
             * <summary>Constructs and initializes a worker.</summary>
             *
             * <param name="start">Start.</param>
             * <param name="end">End.</param>
             * <param name="doneEvent">Done event.</param>
             */
            internal Worker(int start, int end, ManualResetEvent doneEvent, RVOSimulator simulator, bool looped)
            {
                start_ = start;
                end_ = end;
                doneEvent_ = doneEvent;
                simulator_ = simulator;
                this.looped = looped;
            }

            /**
             * <summary>Performs a simulation step.</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void step(object obj)
            {
                for (int agentNo = start_; agentNo < end_; ++agentNo)
                {
                    simulator_.agents_[agentNo].computeNeighbors();

                }


                doneEvent_.Set();
            }

            internal void step2(object obj)
            {
                for (int agentNo = start_; agentNo < end_; ++agentNo)
                {
                    simulator_.agents_[agentNo].applymodel();
                }


                doneEvent_.Set();
            }


            /**
             * <summary>updates the two-dimensional position and
             * two-dimensional velocity of each agent.</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void update(object obj)
            {
                for (int agentNo = start_; agentNo < end_; ++agentNo)
                {
                    simulator_.agents_[agentNo].update();
                }

                doneEvent_.Set();
            }
        }

        internal float RVO_EPSILON = 0.00001f;
        internal IList<Agent> agents_;
        internal IList<Agent> virtual_and_agents_;
        internal Agent defaultAgent_;
        internal float globalTime_;
        internal KdTree kdTree_;
        internal IList<Obstacle> obstacles_;
        public float timeStep_;
        internal int RVO_ERROR = Int32.MaxValue;
        internal IList<GroupAgent> groupAgents_;
        private ManualResetEvent[] doneEvents_;
        private Worker[] workers_;
        private int numWorkers_;
        internal int pas = 0;
        public int start = System.DateTime.Now.Millisecond;
        public bool save = false;

        public RVOSimulator()
        {
            agents_ = new List<Agent>();
            virtual_and_agents_ = new List<Agent>();
            defaultAgent_ = new RVOAgent(this);
            globalTime_ = 0.0f;
            obstacles_ = new List<Obstacle>();
            timeStep_ = 0.1f;
            kdTree_ = new KdTree(this);
            groupAgents_ = new List<GroupAgent>();
            numWorkers_ = 40;
        }

        public RVOSimulator(float timeStep, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {

            agents_ = new List<Agent>();
            virtual_and_agents_ = new List<Agent>();
            globalTime_ = 0.0f;
            obstacles_ = new List<Obstacle>();
            timeStep_ = timeStep;
            kdTree_ = new KdTree(this);
            defaultAgent_ = new RVOAgent(this);
            groupAgents_ = new List<GroupAgent>();
            numWorkers_ = 40;

            defaultAgent_.maxNeighbors_ = maxNeighbors;
            defaultAgent_.maxSpeed_ = maxSpeed;
            defaultAgent_.neighborDist_ = neighborDist;
            defaultAgent_.radius_ = radius;
            defaultAgent_.timeHorizon_ = timeHorizon;
            defaultAgent_.timeHorizonObst_ = timeHorizonObst;
            defaultAgent_.velocity_ = velocity;
        }

        public RVOSimulator(float timeStep, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed)
        {

            agents_ = new List<Agent>();
            virtual_and_agents_ = new List<Agent>();
            globalTime_ = 0.0f;
            obstacles_ = new List<Obstacle>();
            timeStep_ = timeStep;
            kdTree_ = new KdTree(this);
            defaultAgent_ = new RVOAgent(this);
            groupAgents_ = new List<GroupAgent>();
            numWorkers_ = 40;

            defaultAgent_.maxNeighbors_ = maxNeighbors;
            defaultAgent_.maxSpeed_ = maxSpeed;
            defaultAgent_.neighborDist_ = neighborDist;
            defaultAgent_.radius_ = radius;
            defaultAgent_.timeHorizon_ = timeHorizon;
            defaultAgent_.timeHorizonObst_ = timeHorizonObst;
            defaultAgent_.velocity_ = new Vector2();
        }



        public int getAgentNumAgentNeighbors(int agentNo)
        {
            return agents_[agentNo].agentNeighbors_.Count;
        }

        public int getAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo].agentNeighbors_[neighborNo].Value.id_;
        }

        public int getAgentObstacleNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo].obstacleNeighbors_[neighborNo].Value.id_;
        }

        public int getAgentNumObstacleNeighbors(int agentNo)
        {
            return agents_[agentNo].obstacleNeighbors_.Count;
        }

        public float getAgentMaxSpeed()
        {
            return defaultAgent_.maxSpeed_;
        }

        public int getAgentNumORCALines(int agentNo)
        {
            return agents_[agentNo].orcaLines_.Count;
        }

        public Line getAgentORCALine(int agentNo, int lineNo)
        {
            return agents_[agentNo].orcaLines_[lineNo];
        }

        public int addAgent(Vector2 position, int type, bool follow, bool group)
        {
            if (defaultAgent_ == null)
            {
                return RVO_ERROR;
            }
            Agent agent;
            switch (type)
            {
                case 0:
                    {
                        agent = new RVOAgent(this);
                    }
                    break;
                case 1:
                    {
                        agent = new HelbingAgent(this, 0);
                    }
                    break;
                case 2:
                    {
                        agent = new HelbingAgent(this, 1);
                    }
                    break;
                default:
                    return RVO_ERROR;
            }

            agent.position_ = position;

            agent.maxNeighbors_ = defaultAgent_.maxNeighbors_;
            agent.maxSpeed_ = defaultAgent_.maxSpeed_;
            agent.neighborDist_ = defaultAgent_.neighborDist_;
            agent.radius_ = defaultAgent_.radius_;

            agent.timeHorizon_ = defaultAgent_.timeHorizon_;
            agent.timeHorizonObst_ = defaultAgent_.timeHorizonObst_;
            agent.velocity_ = defaultAgent_.velocity_;

            agent.follows_ = follow;
            agent.considerGroups_ = group;

            agent.id_ = agents_.Count;

            agents_.Add(agent);
            return agents_.Count - 1;
        }

        public int addAgent(Vector2 position, int type, bool follow, bool group,
            float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            if (defaultAgent_ == null)
            {
                return RVO_ERROR;
            }
            Agent agent;
            switch (type)
            {
                case 0:
                    {
                        agent = new RVOAgent(this);
                    }
                    break;
                case 1:
                    {
                        agent = new HelbingAgent(this, 0);
                    }
                    break;
                case 2:
                    {
                        agent = new HelbingAgent(this, 1);
                    }
                    break;
                default:
                    return RVO_ERROR;
            }

            agent.position_ = position;
            agent.maxNeighbors_ = maxNeighbors;
            agent.maxSpeed_ = maxSpeed;
            agent.neighborDist_ = neighborDist;
            agent.radius_ = radius;
            agent.timeHorizon_ = timeHorizon;
            agent.timeHorizonObst_ = timeHorizonObst;
            agent.velocity_ = velocity;

            agent.follows_ = follow;
            agent.considerGroups_ = group;

            agent.id_ = agents_.Count;

            agents_.Add(agent);

            return agents_.Count - 1;
        }

        public int addVirtualAgent(int model_no, Vector2 position)
        {

            Agent model = agents_[model_no];
            Agent agent;
            agent = new RVOAgent(this);
            agent.position_ = position;
            agent.radius_ = model.radius_;
            agent.velocity_ = model.velocity_;
            agent.velocityBuffer_ = model.velocityBuffer_;

            agent.follows_ = model.follows_;
            agent.considerGroups_ = model.considerGroups_;

            agent.id_ = virtual_and_agents_.Count;

            virtual_and_agents_.Add(agent);

            return virtual_and_agents_.Count - 1;
        }

        public void initialize_virtual_and_agents()
        {
            virtual_and_agents_ = new List<Agent>();
            foreach(Agent a in agents_)
            {
                virtual_and_agents_.Add(a);
            }

        }

        public void removeAgent(int i)
        {
            agents_.RemoveAt(i);
        }

        public int addObstacle(IList<Vector2> vertices)
        {
            if (vertices.Count < 2)
            {
                return RVO_ERROR;
            }

            int obstacleNo = obstacles_.Count;

            for (int i = 0; i < vertices.Count; ++i)
            {
                Obstacle obstacle = new Obstacle();
                obstacle.point_ = vertices[i];
                if (i > 0)
                {
                    obstacle.previous_ = obstacles_[obstacleNo - 1];
                    obstacle.previous_.next_ = obstacle;


                }
                if (i == vertices.Count - 1)
                {
                    obstacle.next_ = obstacles_[obstacleNo - 1];
                    obstacle.next_.previous_ = obstacle;
                }
                obstacle.direction_ = Vector2.normalize(vertices[(i == vertices.Count - 1 ? 0 : i + 1)] - vertices[i]);

                if (vertices.Count == 2)
                {
                    obstacle.convex_ = true;
                }
                else
                {
                    obstacle.convex_ = (RVOMath.leftOf(vertices[(i == 0 ? vertices.Count - 1 : i - 1)], vertices[(i == vertices.Count - 1 ? 0 : i + 1)], vertices[i]) >= 0);
                }                                    // TO DO - Fix leftOf(dunno what is do) 

                obstacle.id_ = obstacles_.Count;


                obstacles_.Add(obstacle);
                obstacleNo = obstacles_.Count;
            }

            return obstacleNo;
        }


        public float DoStep(Boolean looped)
        {
            
            
                if (workers_ == null || agents_.Count > workers_[workers_.Length-1].end_)
                {
                    workers_ = new Worker[numWorkers_];
                    doneEvents_ = new ManualResetEvent[workers_.Length];

                    for (int block = 0; block < workers_.Length; ++block)
                    {
                        doneEvents_[block] = new ManualResetEvent(false);
                        workers_[block] = new Worker(block * getNumAgents() / workers_.Length, (block + 1) * getNumAgents() / workers_.Length, doneEvents_[block], this, looped);
                    }
                }

                kdTree_.buildAgentTree(looped);

                for (int block = 0; block < workers_.Length; ++block)
                {
                    doneEvents_[block].Reset();
                    ThreadPool.QueueUserWorkItem(workers_[block].step);
                }

                WaitHandle.WaitAll(doneEvents_);
                detectGroups(looped);
                for (int block = 0; block < workers_.Length; ++block)
                {
                    doneEvents_[block].Reset();
                    ThreadPool.QueueUserWorkItem(workers_[block].step2);
                }

                WaitHandle.WaitAll(doneEvents_);

                for (int block = 0; block < workers_.Length; ++block)
                {
                    doneEvents_[block].Reset();
                    ThreadPool.QueueUserWorkItem(workers_[block].update);
                }

                WaitHandle.WaitAll(doneEvents_);
            if(save)
                new DataThread(this, looped, agents_[0].follows_);
            globalTime_ += timeStep_;
            if (looped)
                virtual_and_agents_ = new List<Agent>();
            pas++;
            return globalTime_;
        }

        /**
         * <summary>C++ version of doStep. Aborted because optimisation used are not supported in C# using .Net Framework 3.5</summary>
         *
         * <param name="looped">Determine wether the simulation loop or not. </param>
         */
        /*
        public void doStep(bool looped)
        {

            kdTree_.buildAgentTree();
            for (int i = 0; i < agents_.Count; i++)
            {
                agents_[i].computeNeighbors();
            }

            detectGroups(looped);
            for (int i = 0; i < agents_.Count; ++i)
            {
                agents_[i].applymodel();
            }


            for (int i = 0; i < agents_.Count; ++i)
            {
                agents_[i].update();

            }

            globalTime_ += timeStep_;
            DataThread dt = new DataThread(this, looped, agents_[0].follows_);
            
        }*/

        public int getAgentMaxNeighbors(int agentNo)
        {
            return agents_[agentNo].maxNeighbors_;
        }

        public float getAgentMaxSpeed(int agentNo)
        {
            return agents_[agentNo].maxSpeed_;
        }

        public float getAgentNeighborDist(int agentNo)
        {
            return agents_[agentNo].neighborDist_;
        }

        internal Agent getAgent(int agentNo)
        {
            return agents_[agentNo];
        }

        public Vector2 getAgentPosition(int agentNo)
        {
            return agents_[agentNo].position_;
        }

        public int getAgentLeaderNo(int agentNo)
        {
            int toReturn = -1;
            Agent leader = agents_[agentNo].leader_;
            if (leader != null)
                toReturn = leader.id_;
            return toReturn;
        }

        public GroupAgent getAgentGroup(int agentNo)
        {
            return agents_[agentNo].groupBelongingTo_;
        }

        public float getAgentLocalDensity(int agentNo, bool looped, float obs_distance)
        {
            // Deal with looped scenarios
            IList<Agent> agents;
            if (looped)
                agents = virtual_and_agents_;
            else
                agents = agents_;

            int cpt = 0;
            Vector2 position = getAgentPosition(agentNo);
            for (int i = 0; i < agents.Count; i++)
            {
                if (Vector2.abs(position - agents[i].position_) < obs_distance)
                    cpt++;
            }
            return cpt / ((float)Math.PI * obs_distance * obs_distance);
        }


        internal float getAgentCptDist(int agentNo)
        {
            return agents_[agentNo].cpt_dist;
        }

        internal int getAgentCptTime(int agentNo)
        {
            return agents_[agentNo].cpt_time;
        }

        public float getAgentDistanceWithCloserAgent(int agentNo, bool looped)
        {
            // Deal with looped scenarios
            IList<Agent> agents;
            if (looped)
                agents = virtual_and_agents_;
            else
                agents = agents_;

            float closer_distance = Single.PositiveInfinity;
            float distance;
            Vector2 agent_position = agents_[agentNo].position_;
            for (int i = 0; i < agents.Count; i++)
            {
                distance = Vector2.abs(agent_position - agents[i].position_);
                if (distance > RVO_EPSILON && distance < closer_distance)

                    closer_distance = distance;
            }
            return closer_distance;
        }

        public IList<int> getGroupMembers(GroupAgent ga)
        {
            IList<Agent> groupAgents = ga.getAgents();
            IList<int> toReturn = new List<int>();
            foreach (Agent a in groupAgents)
            {
                toReturn.Add(a.id_);
            }
            return toReturn;
        }

        public Vector2 getAgentPrefVelocity(int agentNo)
        {
            return agents_[agentNo].prefVelocity_;
        }

        public Vector2 getAgentGoal(int agentNo)
        {
            return agents_[agentNo].goal_;
        }

        public float getAgentRadius(int agentNo)
        {
            return agents_[agentNo].radius_;
        }

        public float getAgentTimeHorizon(int agentNo)
        {
            return agents_[agentNo].timeHorizon_;
        }

        public float getAgentTimeHorizonObst(int agentNo)
        {
            return agents_[agentNo].timeHorizonObst_;
        }

        public Vector2 getAgentVelocity(int agentNo)
        {
            return agents_[agentNo].velocity_;
        }

        public Vector2 getAgentNewVelocity(int agentNo)
        {
            return agents_[agentNo].newVelocity_;
        }

        public Vector2 getAgentAcceleration(int agentNo)
        {
            return agents_[agentNo].acceleration_;
        }


        public Pair<float, float> getAgentTerrainMaxSpeeds(int agentNo)
        {
            return agents_[agentNo].terrain_max_speeds_;
        }

        public float getGlobalTime()
        {
            return globalTime_;
        }

        public int getNumAgents()
        {
            return agents_.Count;
        }

        public int getNumObstacleVertices()
        {
            return obstacles_.Count;
        }

        public Vector2 getObstacleVertex(int vertexNo)
        {
            return obstacles_[vertexNo].point_;
        }

        public int getNextObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].next_.id_;
        }

        public int getPrevObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].previous_.id_;
        }
        public float getAgentAngle(int agent)
        {
            return (float)Math.Acos(Vector2.dot(agents_[agent].velocity_, agents_[agent].velocityBuffer_[agents_[agent].velocityBuffer_.Count - 1]));

        }

        public float getTimeStep()
        {
            return timeStep_;
        }

        public float getDefaultRadius()
        {
            return defaultAgent_.radius_;
        }

        public void processObstacles()
        {
            kdTree_.buildObstacleTree();
        }

        public bool queryVisibility(Vector2 point1, Vector2 point2, float radius)
        {
            return kdTree_.queryVisibility(point1, point2, radius);
        }

        public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst,
            float radius, float maxSpeed, Vector2 velocity)
        {
            if (defaultAgent_ == null)
            {
                defaultAgent_ = new RVOAgent(this);
            }

            defaultAgent_.maxNeighbors_ = maxNeighbors;
            defaultAgent_.maxSpeed_ = maxSpeed;
            defaultAgent_.neighborDist_ = neighborDist;
            defaultAgent_.radius_ = radius;
            defaultAgent_.timeHorizon_ = timeHorizon;
            defaultAgent_.timeHorizonObst_ = timeHorizonObst;
            defaultAgent_.velocity_ = velocity;
        }

        public void setAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            agents_[agentNo].maxNeighbors_ = maxNeighbors;
        }

        public void setAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            agents_[agentNo].maxSpeed_ = maxSpeed;
        }

        public void setAgentTerrainMaxSpeeds(int agentNo, Pair<float, float> terrain_max_speeds_)
        {
            agents_[agentNo].terrain_max_speeds_ = terrain_max_speeds_;
        }

        public void setAgentNeighborDist(int agentNo, float neighborDist)
        {
            agents_[agentNo].neighborDist_ = neighborDist;
        }

        public void setAgentPosition(int agentNo, Vector2 position)
        {
            agents_[agentNo].position_ = position;
        }

        public void setAgentPrefVelocity(int agentNo, Vector2 prefVelocity)
        {
            agents_[agentNo].prefVelocity_ = prefVelocity;
        }

        public void setAgentGoal(int agentNo, Vector2 goal)
        {
            agents_[agentNo].goal_ = goal;
        }

        public void setAgentRadius(int agentNo, float radius)
        {
            agents_[agentNo].radius_ = radius;
        }

        public void setAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            agents_[agentNo].timeHorizon_ = timeHorizon;
        }

        public void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            agents_[agentNo].timeHorizonObst_ = timeHorizonObst;
        }

        public void setAgentVelocity(int agentNo, Vector2 velocity)
        {
            agents_[agentNo].velocity_ = velocity;
        }

        public void setTimeStep(float timeStep)
        {
            timeStep_ = timeStep;
        }
        
        public void  reinitializeOutputVariables(int agentNo)
        {
            agents_[agentNo].cpt_dist = 0;
            agents_[agentNo].cpt_time = 0;
        }
        
        public void detectGroups(bool looped)
        {
            // Deal with looped scenarios
            IList<Agent> agents;
            if (looped)
                agents = virtual_and_agents_;
            else
                agents = agents_;

            // Remove groups from last step
            for (int i = 0; i < agents_.Count; i++)
            {
                agents_[i].groupBelongingTo_ = null;
            }

            groupAgents_.Clear();

            for (int i = 0; i < agents.Count - 1; i++)
            {
                Agent agent = agents[i];
                for (int neighbor_id = 0; neighbor_id < agent.agentNeighbors_.Count; ++neighbor_id)
                {
                    Agent toCompare = agents[agent.agentNeighbors_[neighbor_id].Value.id_];
                    float distance = Vector2.abs(agent.position_ - toCompare.position_);
                    float delta_angle = Math.Abs(Vector2.angle(agent.velocity_) - Vector2.angle(toCompare.velocity_));
                    float delta_speed = Math.Max(Vector2.abs(agent.velocity_), Vector2.abs(toCompare.velocity_)) - Math.Min(Vector2.abs(agent.velocity_), Vector2.abs(toCompare.velocity_));
                    // Check distance and velocity  raints
                    if (distance < defaultAgent_.radius_ * 5
                        && (delta_angle < Math.PI / 6 || delta_angle > 11 * Math.PI / 6)
                        && delta_speed < 0.2 * Math.Max(agent.maxSpeed_, toCompare.maxSpeed_)
                        )
                    {
                        mergeGroups(agent, toCompare);
                    }
                }
            }
        }

        internal void mergeGroups(Agent agent1, Agent agent2)
        {
            if (agent1.groupBelongingTo_ == null && agent2.groupBelongingTo_ == null)
            {
                // Create new group
                GroupAgent group = new GroupAgent(this);
                group.agents_.Add(agent1);
                group.agents_.Add(agent2);
                agent1.groupBelongingTo_ = group;
                agent2.groupBelongingTo_ = group;
                groupAgents_.Add(group);
            }
            else if (agent1.groupBelongingTo_ == null)
            {
                agent1.groupBelongingTo_ = agent2.groupBelongingTo_;
                agent2.groupBelongingTo_.agents_.Add(agent1);
            }
            else if (agent2.groupBelongingTo_ == null)
            {

                agent2.groupBelongingTo_ = agent1.groupBelongingTo_;
                agent1.groupBelongingTo_.agents_.Add(agent2);
            }
            else if (agent1.groupBelongingTo_ != agent2.groupBelongingTo_)
            {
                // Merge groups
                IList<Agent> agentsList2 = agent2.groupBelongingTo_.agents_;
                foreach (Agent a in agentsList2)
                {
                    Agent agent = a;
                    agent.groupBelongingTo_ = agent1.groupBelongingTo_;
                    agent1.groupBelongingTo_.agents_.Add(agent);

                }
                agentsList2.Clear();
            }
        }

        public void unitTests()
        {

            // Test Compute tangents Points
            setAgentDefaults(15.0f, 10, 1.0f, 10.0f, 0.5f, 2.0f, new Vector2());
            addAgent(new Vector2(-1, 1), 0, true, true);
            addAgent(new Vector2(0, 1), 0, true, true);
            setAgentRadius(1, 1.0f);
            addAgent(new Vector2(0, 0), 0, true, true);
            addAgent(new Vector2(1, 2), 0, true, true);
            addAgent(new Vector2(1, 0), 0, true, true);

            GroupAgent group = new GroupAgent(this);
            Pair<Vector2, Vector2> p1 = group.computeTangentsPoints(getAgent(0), getAgent(1));
            Pair<Vector2, Vector2> p2 = group.computeTangentsPoints(getAgent(2), getAgent(1));
            Pair<Vector2, Vector2> p3 = group.computeTangentsPoints(getAgent(3), getAgent(1));
            Pair<Vector2, Vector2> p4 = group.computeTangentsPoints(getAgent(4), getAgent(1));

            Pair<Vector2, Vector2> p5 = group.computeTangentsPoints(getAgent(1), getAgent(0));
            Pair<Vector2, Vector2> p6 = group.computeTangentsPoints(getAgent(1), getAgent(2));
            Pair<Vector2, Vector2> p7 = group.computeTangentsPoints(getAgent(1), getAgent(3));
            Pair<Vector2, Vector2> p8 = group.computeTangentsPoints(getAgent(1), getAgent(4));

            Console.Write("Compute Tangents Points\n");
            Console.Write("T1" + p1.First + "Expected ~(-1,0.5) - T2" + p1.Second + "Expected ~(-1,1.5)\n");
            Console.Write("T1" + p2.First + "Expected ~(0.5,0) - T2" + p2.Second + "Expected ~(-0.5,0)\n");
            Console.Write("T1" + p3.First + "Expected ~(0.64,2.35) - T2" + p3.Second + "Expected ~(1.35,1.64)\n");
            Console.Write("T1" + p4.First + "Expected ~(1.35,0.35) - T2" + p4.Second + "Expected ~(0.64,-0.35)\n\n");

            Console.Write("T1" + p5.First + "Expected ~(0,1.5) - T2" + p5.Second + "Expected ~(0,0.5)\n");
            Console.Write("T1" + p6.First + "Expected ~(-0.5,1) - T2" + p6.Second + "Expected ~(0.5,1)\n");
            Console.Write("T1" + p7.First + "Expected ~(0.35,0.64) - T2" + p7.Second + "Expected ~(-0.35,1.35)\n");
            Console.Write("T1" + p8.First + "Expected ~(-0.35,0.64) - T2" + p8.Second + "Expected ~(0.35,1.35)\n\n");

            // Test detect groups
            setAgentRadius(1, 0.5f);
            addAgent(new Vector2(1, 0), 0, true, true);

            setAgentPosition(0, new Vector2(0.0f, 3 * getDefaultRadius()));
            setAgentPosition(1, new Vector2(0.0f, 0.0f));
            setAgentPosition(2, new Vector2(-5 * getDefaultRadius(), 0.0f));
            setAgentPosition(3, new Vector2(-5 * getDefaultRadius(), -3 * getDefaultRadius()));
            setAgentPosition(4, new Vector2(3 * getDefaultRadius(), 0.0f));
            setAgentPosition(5, new Vector2(6 * getDefaultRadius(), 0.0f));

            setAgentVelocity(0, new Vector2(0.87f, 0.0f));
            setAgentVelocity(1, new Vector2(1.0f, 0.0f));
            setAgentVelocity(2, new Vector2(0.93f, 0.0f));
            setAgentVelocity(3, Vector2.rotation(new Vector2(0.93f, 0.0f), (float)Math.PI / 6 + 0.1f));
            setAgentVelocity(4, Vector2.rotation(new Vector2(0.93f, 0.0f), (float)Math.PI / 6 - 0.1f));
            setAgentVelocity(5, Vector2.rotation(new Vector2(0.87f, 0.0f), 2 * ((float)Math.PI / 6 - 0.1f)));

            detectGroups(false);
            Console.Write("Detect Groups\n");
            for (int i = 0; i < agents_.Count; i++)
            {
                Console.Write("Agent:" + i + " Address:" + agents_[i] + " group:" + agents_[i].groupBelongingTo_ + "\n");
            }
            GroupAgent group1 = agents_[1].groupBelongingTo_;
            Console.Write("\n Group1 : Agents = ");
            for (int i = 0; i < group1.agents_.Count; i++)
            {
                Console.Write(group1.agents_[i] + " ");
            }
            Console.Write("\n\n");


            // Test Represent group
            setAgentVelocity(0, new Vector2(1, 0));
            detectGroups(false);
            Console.Write("Detect Groups\n");
            for (int i = 0; i < agents_.Count; i++)
            {
                Console.Write("Agent:" + i + " Address:" + agents_[i] + " group:" + agents_[i].groupBelongingTo_ + "\n");
            }
            group1 = agents_[1].groupBelongingTo_;
            Console.Write("\n Group1 : Agents = ");
            for (int i = 0; i < group1.agents_.Count; i++)
            {
                Console.Write(group1.agents_[i] + " ");
            }
            Console.Write("\n\n");

            addAgent(new Vector2(3, 3), 0, true, true);
            SuperAgent sa = group1.RepresentGroup(agents_.Last());
            Console.Write("Represent group\n");
            Console.Write("Position: " + sa.position_ + ".  Velocity: " + sa.velocity_ + ". Radius: " + sa.radius_ + "\n");
            setAgentPosition(agents_.Count - 1, new Vector2(2, 1.5f));
            sa = group1.RepresentGroup(agents_.Last());
            Console.Write("Position: " + sa.position_ + ".  Velocity: " + sa.velocity_ + ". Radius: " + sa.radius_ + "\n");
            setAgentPosition(agents_.Count - 1, new Vector2(0.75f, 0.75f));
            sa = group1.RepresentGroup(agents_.Last());
            Console.Write("Position: " + sa.position_ + ".  Velocity: " + sa.velocity_ + ". Radius: " + sa.radius_ + "\n");
        }
    }



}


