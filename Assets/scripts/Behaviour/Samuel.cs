using CenterSpace.Free;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Troschuetz.Random;
using UnityEngine.UI;
using System.Threading;
using System.IO;

namespace RVO
{
    class Samuel : Scenario
    {
        private class Worker
        {
            private ManualResetEvent doneEvent_;
            internal IList<Agent> agents;
            public Vector2 vit_moy;
            private RVOSimulator simulator_;
            private bool looped;

            /**
             * <summary>Constructs and initializes a worker.</summary>
             *
             * <param name="start">Start.</param>
             * <param name="end">End.</param>
             * <param name="doneEvent">Done event.</param>
             */
            internal Worker( ManualResetEvent doneEvent, RVOSimulator simulator, bool looped)
            {
                agents = new List<Agent>();
                vit_moy = new Vector2();
                doneEvent_ = doneEvent;
                simulator_ = simulator;
                this.looped = looped;
            }

            /**
             * <summary>Performs a simulation step.</summary>
             *
             * <param name="obj">Unused.</param>
             */
             internal void addAgent(Agent a)
            {
                agents.Add(a);
            }

            internal void computeMedVelocity(object obj)
            {
                foreach (Agent a in agents)
                {
                    vit_moy += a.velocity_;
                    vit_moy /= 2;
                }
                doneEvent_.Set();
            }


            internal void clear(object obj)
            {
                agents.Clear();
                doneEvent_.Set();
            }

        }



        public Transform prefab;

        public Samuel() : base() { }
        int ped_num_;
        int model_type_ = 0;
        bool follow_ =true;
        bool group_ = false;
        int fluxes_ = 1;
        float corridor_length_ = 50;
        float corridor_width_ = 10;
        float corridor_angle_ = 0;
        bool loop_ = true;
        float ped_radius_ = 0.35f;
        float time_step = 0.1f;
        List<Color> colors = new List<Color>();
        List<int> step_stop = new List<int>();
        public Toggle follow_but_;
        public Toggle save_but_;
        public Slider camera_height_;
        private IList<Worker> workers;
        private ManualResetEvent[] doneEvents_;
        public new string name;

        void Start()
        {

            camera_height_.minValue = 10;
            camera_height_.maxValue = 80;
            camera_height_.value = Camera.main.transform.position.y;
            agents = new List<Transform>();
            Application.targetFrameRate = 20;
            Application.runInBackground = true;
            sim_.setAgentDefaults(1f, 10, 1, 1, ped_radius_, 2, new Vector2(0, 0));
            ped_num_ = (int)(1.5f * corridor_width_ * (corridor_length_ + 10));
            //ped_num_ = 250;
            sim_.setTimeStep(time_step);
            transform.localScale = new Vector3(corridor_length_ / 10, 1, corridor_width_ / 10);
            transform.position = new Vector3(corridor_length_ / 2, 0, corridor_width_ / 2);
            setupScenario();
            follow_but_.isOn = follow_;

            int worker_num = (int)((corridor_length_-20) / (8 * ped_radius_)) + 2;
            initializeWorker(worker_num);


            colors.Add(new Color(0.000f, 0.000f, 0.804f));
            colors.Add(new Color(0.118f, 0.565f, 1.000f));
            colors.Add(new Color(0.251f, 0.878f, 0.816f));
            colors.Add(new Color(0.400f, 0.804f, 0.667f));
            colors.Add(new Color(0.604f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.096f));
            colors.Add(new Color(0.94f, 0.804f, 0.10f));

            name = "Data";
            string num = sim_.getNumAgents().ToString();
            name += num + "wave";

            name += "_agents.csv";
            if (!File.Exists(name))
            {
                File.Create(name).Dispose();
                string tmp = "";
                for(int i =1; i < workers.Count - 2; ++i)
                {
                    tmp += "moyenne des vitesses de la zone " + i + "\t";
                }
                using (TextWriter tw = new StreamWriter(name))
                {
                    tw.WriteLine(tmp);
                }
            }

            sim_.processObstacles();
            sim_.kdTree_.buildAgentTree();
        }

        private void initializeWorker(int worker_num)
        {
            workers = new Worker[worker_num];
            doneEvents_ = new ManualResetEvent[workers.Count];

            for (int block = 0; block < workers.Count; ++block)
            {
                doneEvents_[block] = new ManualResetEvent(false);
                workers[block] = new Worker(doneEvents_[block], sim_, false);

            }


            updateWorker(worker_num);
        }

        private void updateWorker(int worker_num)
        {
            foreach (Agent a in sim_.agents_)
            {
                if (a.position_.x_ <= 10)
                {
                    workers[0].addAgent(a);
                }

                else if (a.position_.x_ >= corridor_length_ - 10)
                {
                    workers[worker_num - 1].addAgent(a);
                }
                else
                {
                    int worker_id = (int)((a.position_.x_ - 10) / (8 * ped_radius_));
                    workers[worker_id + 1].addAgent(a);
                }
            }
        }


        // Update is called once per frame
        void Update()
        {
            for (int block = 0; block < workers.Count; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers[block].clear);
            }

            WaitHandle.WaitAll(doneEvents_);
            updateWorker(workers.Count);
            if (!reachedGoal())
            {
                setPreferredVelocities();
                doStep();
                setAgentsProperties();
                /* Output the current global time. */
                //print(Simulator.Instance.getGlobalTime());

                if (follow_but_.isOn != follow_)
                {
                    follow_ = follow_but_.isOn;

                    for (int i = 0; i < getNumAgents(); ++i)
                    {
                        sim_.agents_[i].follows_ = follow_;
                    }
                }
                if (save_but_.isOn != sim_.save)
                {
                    sim_.save = save_but_.isOn;

                }

                Vector3 pos3 = Camera.main.transform.position;
                Camera.main.transform.position = new Vector3(pos3.x, camera_height_.value, pos3.z);



                for (int i = 0; i < getNumAgents(); ++i)
                {
                    Vector2 position = sim_.getAgentPosition(i);
                    if (!(sim_.agents_[i].velocity_ == new Vector2(0, 0)))
                    {
                        step_stop[i] = 0;
                    }
                    else
                    {
                        step_stop[i]++;
                    }

                    if (step_stop[i] > 6)
                    {
                        sim_.agents_[i].position_ = new Vector2(-5, position.y());
                        agents[i].transform.position = new Vector3(-5, 0f, position.y());


                    }
                    else
                    {
                        agents[i].transform.position = new Vector3(position.x(), 0f, position.y());

                    }
                     RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                      agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
                    setColor(i);
                }
            }
            else
            {
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    agents[i].transform.GetComponent<Rigidbody>().isKinematic = true;
                }
            }

            for (int block = 0; block < workers.Count; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers[block].computeMedVelocity);
            }

            WaitHandle.WaitAll(doneEvents_);

            String tmp = "";
            for (int i = 1; i < workers.Count-2; i++)
            {
                tmp += Vector2.abs(workers[i].vit_moy)+ "\t";

            }
            using (TextWriter tw = new StreamWriter(name, true))
            {
                tw.WriteLine(tmp);
            }
        }



        void setColor(int i)
        {
            float max_vel = sim_.getAgentMaxSpeed();
            if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= max_vel / 8)
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[0];
            }
            else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 2 * max_vel / 8)
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[1];
            }
            else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 3 * max_vel / 8)
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[2];
            }
            else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 4 * max_vel / 8)
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[3];
            }
            else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 5 * max_vel / 8)
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[4];
            }
            else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 6 * max_vel / 8)
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[5];
            }
            else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 7 * max_vel / 8)
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[6];
            }
            else
            {
                agents[i].GetComponent<MeshRenderer>().material.color = colors[7];
            }
        }




        void setCorridor()
        {// Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
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


             IList<Vector2> begin_side = new List<Vector2> {
             Vector2.rotation(new Vector2(-0, -50), corridor_angle_),
             Vector2.rotation(new Vector2(-0 , corridor_width_+50), corridor_angle_),
             Vector2.rotation(new Vector2(-10, corridor_width_ + 50.0f), corridor_angle_),
             Vector2.rotation(new Vector2(-10, 0), corridor_angle_) };
             sim_.addObstacle(begin_side);
            // Process obstacles so that they are accounted for in the simulation.
            sim_.processObstacles();
        }



        void placeAgents()
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
                float x = (float)x_distribution.NextDouble() * corridor_length_ % corridor_length_ ;
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
                            if (ped < ped_num_ / 2)
                            {
                                Vector2 corridor_end = new Vector2(corridor_length_ + 1, y);
                                corridor_end = Vector2.rotation(corridor_end, corridor_angle_);
                                sim_.setAgentGoal(ped, corridor_end);
                            }
                            else
                            {
                                Vector2 corridor_start = new Vector2(-100, y);
                                corridor_start = Vector2.rotation(corridor_start, corridor_angle_);
                                sim_.setAgentGoal(ped, corridor_start);
                            }
                            break;
                        }
                    default:
                        break;
                }
            }
        }


        void setAgentsProperties()
        {
            for (int i = 0; i < sim_.getNumAgents(); ++i)
            {  // Set Agent Goal
                Vector2 pos = sim_.getAgentPosition(i);
                Vector2 goal = sim_.getAgentGoal(i);
                // Position in the corridor referential
                Vector2 local_pos = Vector2.rotation(pos, -corridor_angle_);
                Vector2 local_goal = Vector2.rotation(goal, -corridor_angle_);
                // Set agent goal
                Vector2 new_goal = new Vector2(local_goal.x(), local_pos.y());
                // Back to world's referential
                new_goal = Vector2.rotation(new_goal, corridor_angle_);
                // Set goal
                sim_.setAgentGoal(i, new_goal);


                if (local_pos.x() > corridor_length_ - 5)
                {
                    if (Vector2.abs(sim_.getAgentVelocity(i)) > (Vector2.abs(workers[0].vit_moy)))
                    {
                        sim_.setAgentVelocity(i, workers[0].vit_moy);
                    }
                }


                // Set Agent Position (looped corridor)
                // If the agent as reached the end of the corridor (case 1)
                if (local_pos.x() >= corridor_length_ && local_goal.x() > corridor_length_)
                {
                    // Put at the start of the corridor
                    Vector2 new_pos = new Vector2(local_pos.x() - (corridor_length_), local_pos.y());
                    // Back to world's referential
                    new_pos = Vector2.rotation(new_pos, corridor_angle_);
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
                    Vector2 new_pos = new Vector2(pos.x_, rand.Next((int)corridor_width_ + 1));
                    // Back to world's referential
                    new_pos = Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, follow_);
                    // Reinitialize data
                    sim_.reinitializeOutputVariables(i);
                }

                // If the agent as reached the end of the corridor (case 2)
                if (local_pos.x() < 0 && local_goal.x() < 0)
                {
                    // Put at the start of the corridor
                    Vector2 new_pos = new Vector2(local_pos.x() + corridor_length_, local_pos.y());
                    // Back to world's referential
                    new_pos = Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, following_behavior_);
                    // Reinitialize data
                    sim_.reinitializeOutputVariables(i);
                }

            }
        }

        private void setVelocity(int i, RVO.Vector2 vector2)
        {
            sim_.agents_[i].velocity_ = vector2;
        }

        public override void setupScenario()
        {
            setCorridor();
            placeAgents();


        }
    }
}
