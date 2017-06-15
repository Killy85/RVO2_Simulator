using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
namespace RVO
{
    class Takeover : Scenario
    {
        public Transform agent_vert;
        float corridor_lenght = 50;
        float corridor_width = 10f;
        float ped_radius_ = 10f;
        String name2;

        float timeStep = 0.05f;
        new String name;
        IList<Color> colors = new List<Color>();
        int pas = 0;
        private int nb_pas=1500;

        public Takeover() : base() { }

        public override void setupScenario()
        {
            // dépassement
            RVO.Vector2 position = new RVO.Vector2(5, 5f);
            // face à face
            //RVO.Vector2 position = new RVO.Vector2(25, 5f);
            sim_.addAgent(position, 0, false, false, 1f, 10, 2f, 2f, 0.35f,1f, new RVO.Vector2(0, 0));
            // dépassement
            sim_.setAgentGoal(0, new RVO.Vector2(corridor_lenght * 1.5f, 5));
            // face à face
            //sim_.setAgentGoal(0, new RVO.Vector2(0,5));

            position = new RVO.Vector2(0, 5.1f);
            sim_.addAgent(position, 0,true, false, 1f, 10,2f, 2f, 0.35f, 4f, new RVO.Vector2(0, 0));
            sim_.setAgentGoal(1, new RVO.Vector2(corridor_lenght * 1.5f, 5.1f));


            colors.Add(new Color(0.000f, 0.000f, 0.804f));
            colors.Add(new Color(0.118f, 0.565f, 1.000f));
            colors.Add(new Color(0.251f, 0.878f, 0.816f));
            colors.Add(new Color(0.400f, 0.804f, 0.667f));
            colors.Add(new Color(0.604f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.096f));
            colors.Add(new Color(0.94f, 0.804f, 0.10f));
        }

        // Use this for initialization
        void Start()
        {
            Application.runInBackground = true;
            Application.targetFrameRate = 30;
            agents = new List<Transform>();

            name = "test" + timeStep + ".csv";
            name2 = "dataAgent.csv"; 
            if (!File.Exists(name))
                File.Create(name).Dispose();
            using (TextWriter tw = new StreamWriter(name))
            {
                tw.WriteLine(" Pas \t 0: Position X \t 0: Position Y \t 1: Position X \t 1: Position Y \t 0: Velocity X \t 0:Velocity Y \t 1: Velocity X \t 1:Velocity Y" +
                    " \t 0: Acceleration X \t 0: Acceleration  \t 1: Acceleration X \t 1: Acceleration Y \t 0: Delta Px \t 0:Delta Py \t 1: Delta Px \t 1:Delta Py   \t 0:Angle  \t 1:Angle \t 0:AgentLeaderNo \t 1:AgentLeaderNo");
            }
            
            if (!File.Exists(name2))
                File.Create(name2).Dispose();
            using (TextWriter tw = new StreamWriter(name2))
            {
                tw.WriteLine("Agent 0 position x \t Agent 0 position y \t Agent 0 leader \t Agent 1 position x \t Agent 1 position y  \t Agent 1 leader ");
            }

            sim_.setTimeStep(timeStep);

            sim_.setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f, new Vector2(0.0f, 0.0f));

            setupScenario();

            for (int i = 0; i < getNumAgents(); ++i)
            {
                addAgent(agent_vert, new Vector3(sim_.getAgentPosition(i).x_, 0, sim_.getAgentPosition(i).y_));
                float radius = sim_.getAgentRadius(i);
                agents[i].transform.localScale = new Vector3(radius, radius, radius);
            }


        }

        void setColor(int i)
        {
            float max_vel = sim_.getAgentMaxSpeed(i);
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

        // Update is called once per frame
        void Update()
        {
            if (! reachedGoal())
            {
                setPreferredVelocities();
                doStep();
             
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    Vector2 position = getPosition(i);
                    agents[i].transform.position = new Vector3(position.x(), 0.5f, position.y());
                    /*  RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                      agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));*/
                    /*  RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                      agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));*/
                    setColor(i);
                    float key = -1f;
                    if (sim_.getAgent(i).agentNeighbors_.Count > 0)
                        key = sim_.getAgent(i).agentNeighbors_[0].Key;

                  

                    pas = pas + i;
                }
                //Debug.Log("Distance " + Math.Sqrt(Math.Pow(agents[0].position.x - agents[1].position.x,2) + Math.Pow(agents[0].position.y - agents[1].position.y,2)));
                //Debug.Log("Vitesse agent 1 " + Math.Sqrt(Math.Pow(sim_.getAgentVelocity(1).x_,2) + Math.Pow(sim_.getAgentVelocity(1).y_,2)));
                using (TextWriter tw = new StreamWriter(name2, true))
                {
                    tw.WriteLine( sim_.getAgent(0).position_.x_ + "\t" + sim_.getAgent(0).position_.y_ + "\t" + sim_.getAgent(0).agentNeighbors_.Count + "\t" + sim_.getAgent(1).position_.x_ + "\t" + sim_.getAgent(1).position_.y_ + "\t" + sim_.getAgent(1).agentNeighbors_.Count );
                }


                Agent neighbor = sim_.getAgent(1);
                Vector2 relative_pos;
                float alpha;
                Vector2 local_relative_pos = new Vector2();
                Agent neighbor2 = sim_.getAgent(0); ;
                Vector2 relative_pos2;
                float alpha2;
                Vector2 local_relative_pos2 = new Vector2();
                relative_pos = neighbor.position_ - sim_.getAgent(0).position_;
                alpha = Vector2.angle(sim_.getAgent(0).velocity_);
                local_relative_pos = Vector2.rotation(relative_pos, -alpha);
                relative_pos2 = neighbor2.position_ - sim_.getAgent(1).position_;
                alpha2 = Vector2.angle(sim_.getAgent(1).velocity_);
                local_relative_pos2 = Vector2.rotation(relative_pos2, -alpha2);


                using (TextWriter tw = new StreamWriter(name, true))
                {
                    tw.WriteLine(pas + "\t" + sim_.getAgentPosition(0).x() + "\t" + sim_.getAgentPosition(0).y() + "\t" + sim_.getAgentPosition(1).x() + "\t" + sim_.getAgentPosition(1).y()
                        + "\t" + sim_.getAgentVelocity(0).x() + "\t" + sim_.getAgentVelocity(0).y() + "\t" + sim_.getAgentVelocity(1).x() + "\t" + sim_.getAgentVelocity(1).y() + 
                        "\t" + sim_.getAgentAcceleration(0).x() + "\t" + sim_.getAgentAcceleration(0).y() + "\t" + sim_.getAgentAcceleration(1).x() + "\t" + sim_.getAgentAcceleration(1).y() + "\t"
                    + local_relative_pos.x() + "\t" + local_relative_pos.y() + "\t"+ local_relative_pos2.x() + "\t" + local_relative_pos2.y() + "\t"
                    + Vector2.angle(sim_.agents_[0].velocity_) * (180 / Math.PI) + "\t"
                    + Vector2.angle(sim_.agents_[1].velocity_) * (180 / Math.PI) + "\t" + sim_.getAgentLeaderNo(0) + "\t" + sim_.getAgentLeaderNo(1));
                }
                
            }
            else
            {

                for (int i = 0; i < getNumAgents(); ++i)
                {
                    agents[i].GetComponent<Rigidbody>().isKinematic = true;
                }
            }

        }
    }
}

