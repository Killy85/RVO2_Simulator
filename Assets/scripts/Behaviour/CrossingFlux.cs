using RVO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Troschuetz.Random;

namespace RVO
{
    internal class CrossingFlux : Scenario
    {


        public CrossingFlux() : base() { }
        int ped_num_;
        int model_type_ = 0;
        bool follow_ = false;
        bool group_ = true;
        int fluxes_ = 2;
        float corridor_length_ = 50;
        float corridor_width_ = 10;
        float corridor_angle_ = 0;
        bool loop_ = true;
        float ped_radius_ = 0.35f;
        float time_step = 0.1f;
        List<int> step_stop = new List<int>();
        /*public Toggle follow_but_;
        public Toggle save_but_;
        public Toggle hum_but_;*/
        public Boolean human_prefab = false;
        public new string name;
        public Transform prefab;

        public override void setupScenario()
        {
            setCorridor();
            placeAgents();
        }

        private void setCorridor()
        {
            // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
            // Add corridor right side

            IList<Vector2> right_side = new List<Vector2> {
            Vector2.rotation(new Vector2(corridor_length_ + 100, 0.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, 0.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, -50.0f), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, -50.0f), corridor_angle_)};
            sim_.addObstacle(right_side);
            //Add cooridor left side
            IList<Vector2> left_side = new List<Vector2> {
            Vector2.rotation(new Vector2(-100.0f, corridor_width_), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, corridor_width_), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, corridor_width_ + 50.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, corridor_width_ + 50.0f), corridor_angle_) };
            sim_.addObstacle(left_side);

            // Process obstacles so that they are accounted for in the simulation.
            sim_.processObstacles();
        }

        private void placeAgents()
        {
            NormalDistribution normal = new NormalDistribution();
            normal.Mu = 1.2;
            normal.Sigma = Math.Sqrt(0.3);

            MT19937Generator generator = new MT19937Generator();
            StandardGenerator sg = new StandardGenerator();

            ContinuousUniformDistribution x_distribution = new ContinuousUniformDistribution(generator);
            NormalDistribution y_distribution = new NormalDistribution(generator);
            y_distribution.Mu = corridor_width_ / 2;
            y_distribution.Sigma = sim_.getDefaultRadius();
            for (int ped = 0; ped < ped_num_; ped++)
            {  // Place Agent
                float x = (float)x_distribution.NextDouble() * corridor_length_ % corridor_length_;
                float y = (float)((y_distribution.NextDouble() * corridor_width_) - 9) % corridor_width_;
                
                Vector2 position = new Vector2(x, Math.Abs(y));
                position = Vector2.rotation(position, corridor_angle_);

                sim_.addAgent(position, model_type_, follow_, group_);

                addAgent(prefab, new Vector3(position.x(), 0, position.y()), sim_.getDefaultRadius());


                step_stop.Add(0);

                // Set agent max speeds
                sim_.setAgentMaxSpeed(ped, (float)normal.NextDouble());
                if (sim_.getAgentMaxSpeed(ped) > 2.0f)
                    sim_.setAgentMaxSpeed(ped, 2.0f);
                if (sim_.getAgentMaxSpeed(ped) < 0.3f)
                    sim_.setAgentMaxSpeed(ped, 0.3f);

                // Set agent's goal
                /* Change the switch in case 2 like :
                 * Instantiate 2 Agents (one at each side) in a geometric way
                 * Add color and goals. 
                */
                switch (fluxes_)
                {
                    case 1:
                        {
                            Vector2 corridor_end = new Vector2(corridor_length_ + 100, y);
                            //Vector2 corridor_end = new Vector2(corridor_length_ + 12, y);
                            corridor_end = Vector2.rotation(corridor_end, corridor_angle_);
                            sim_.setAgentGoal(ped, corridor_end);
                            break;
                        }
                    case 2:
                        {
                            if (ped < ped_num_/2)
                            {
                                Vector2 corridor_end = new Vector2(corridor_length_ + 1, y);
                                corridor_end = Vector2.rotation(corridor_end, corridor_angle_);
                                agents[ped].transform.GetComponent<MeshRenderer>().material.color = new Color(1, 0, 0);
                                sim_.setAgentGoal(ped, corridor_end);
                            }
                            else
                            {
                                Vector2 corridor_start = new Vector2(-100, y);
                                corridor_start = Vector2.rotation(corridor_start, corridor_angle_);
                                agents[ped].transform.GetComponent<MeshRenderer>().material.color = new Color(0, 0, 1);
                                sim_.setAgentGoal(ped, corridor_start);
                            }
                            break;
                        }
                    default:
                        break;
                }
            }
        }

        // Use this for initialization
        void Start()
        {
            agents = new List<Transform>();
            Application.targetFrameRate = 60;
            Application.runInBackground = true;
            sim_.setAgentDefaults(1f, 10, 1, 1, ped_radius_, 2, new RVO.Vector2(0, 0));
            ped_num_ = (int)(0.35f * corridor_width_ * (corridor_length_ + 10));
            //ped_num_ = 250;
            sim_.setTimeStep(time_step);
            transform.localScale = new Vector3(corridor_length_ / 10, 1, corridor_width_ / 10);
            transform.position = new Vector3(corridor_length_ / 2, 0, corridor_width_ / 2);
            setupScenario();
            sim_.initialize_virtual_and_agents();
            sim_.processObstacles();
            sim_.kdTree_.buildAgentTree(true);
        }

        // Update is called once per frame
        void Update()
        {
            if (!reachedGoal())
            {

                setAgentsProperties();
                setPreferredVelocities();

                sim_.initialize_virtual_and_agents();
                for (int i = 0; i < getNumAgents(); i++)
                {
                    RVO.Vector2 agent_position = sim_.getAgentPosition(i);
                    RVO.Vector2 p1 = agent_position + new RVO.Vector2(corridor_length_, 0);
                    RVO.Vector2 p2 = agent_position - new RVO.Vector2(corridor_length_, 0);
                    sim_.addVirtualAgent(0, p1);
                    sim_.addVirtualAgent(0, p2);

                }
                doStep(true);


                int totot = getNumAgents();
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    RVO.Vector2 position = sim_.getAgentPosition(i);
                    agents[i].transform.position = new Vector3(position.x(), 0f, position.y());
                    RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
                    if (human_prefab)
                    {

                        if (RVO.Vector2.absSq(sim_.getAgentVelocity(i) * 4) > 1.5f)
                            agents[i].GetComponent<Animator>().CrossFade("mixamo.com", 10, 1);

                        agents[i].GetComponent<Animator>().speed = RVO.Vector2.absSq(sim_.getAgentVelocity(i) * 4);
                    }
                }
            }
            else
            {
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    agents[i].transform.GetComponent<Rigidbody>().isKinematic = true;
                }
            }
        }

        private void setAgentsProperties()
        {
            for (int i = 0; i < sim_.getNumAgents(); ++i)
            {  // Set Agent Goal
                RVO.Vector2 pos = sim_.getAgentPosition(i);
                RVO.Vector2 goal = sim_.getAgentGoal(i);
                // Position in the corridor referential
                RVO.Vector2 local_pos = RVO.Vector2.rotation(pos, -corridor_angle_);
                RVO.Vector2 local_goal = RVO.Vector2.rotation(goal, -corridor_angle_);
                // Set agent goal
                RVO.Vector2 new_goal = new RVO.Vector2(local_goal.x(), local_pos.y());
                // Back to world's referential
                new_goal = RVO.Vector2.rotation(new_goal, corridor_angle_);
                // Set goal
                sim_.setAgentGoal(i, new_goal);

                // Set Agent Position (looped corridor)
                // If the agent as reached the end of the corridor (case 1)
                if (local_pos.x() >= corridor_length_ && local_goal.x() > corridor_length_)
                {
                    // Put at the start of the corridor
                    RVO.Vector2 new_pos = new RVO.Vector2(local_pos.x() - (corridor_length_), local_pos.y());
                    // Back to world's referential
                    new_pos = RVO.Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, follow_);
                    // Reinitialize data
                    sim_.reinitializeOutputVariables(i);
                }
                if (pos.y() > corridor_width_ || pos.y() < 0)
                {
                    System.Random rand = new System.Random();
                    RVO.Vector2 new_pos = new RVO.Vector2(pos.x_, rand.Next((int)corridor_width_ + 1));
                    // Back to world's referential
                    new_pos = RVO.Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, follow_);
                    // Reinitialize data
                    sim_.reinitializeOutputVariables(i);
                }

                // If the agent as reached the end of the corridor (case 2)
                if (local_pos.x() <= 0 && local_goal.x() < 0)
                {
                    // Put at the start of the corridor
                    RVO.Vector2 new_pos = new RVO.Vector2(local_pos.x() + corridor_length_, local_pos.y());
                    // Back to world's referential
                    new_pos = RVO.Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, following_behavior_);
                    // Reinitialize data
                    sim_.reinitializeOutputVariables(i);
                }

            }
        }
    }
}
