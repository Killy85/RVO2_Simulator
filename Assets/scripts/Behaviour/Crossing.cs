using RVO;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

class Crossing : Scenario
{
    /** Those  are the prefabs we will use to instantiate agent and walls **/
    public Transform agent_rose;
    public Transform agent_vert;
    public Transform wall;
    public Toggle group;
    public Toggle avoidance;
    private Boolean group_b;
    private Boolean avoidance_b;


    public Crossing() : base() { }
   


    // Use this for initialization
    void Start()
    {
        group_b = group.isOn;
        avoidance_b = avoidance.isOn;
        Application.targetFrameRate = 60;
        agents = new List<Transform>();
        /* Set up the scenario. */
        setupScenario();
        Instantiate(wall, new Vector3(0, 12.5f, 40.5f), Quaternion.identity);
        Instantiate(wall, new Vector3(0, 12.5f, -39.5f), Quaternion.identity);

        /* Perform (and manipulate) the simulation. */
        for (int i = 0; i < getNumAgents(); ++i)
        {
            RVO.Vector2 position = getPosition(i);
            int mid = (group_b ? getNumAgents() / 2 : 1);

            if (i < mid) { addAgent(agent_rose, new Vector3(position.x(),0f, position.y()),sim_.getDefaultRadius()); }
            else { addAgent(agent_vert, new Vector3(position.x(), 0f, position.y()), sim_.getDefaultRadius()); }
        }
        
    }

    // Update is called once per frame
    void Update()
    {
        if(group_b != group.isOn ||avoidance_b != avoidance.isOn)
        {
            group_b = group.isOn;
            avoidance_b = avoidance.isOn;
            int range = agents.Count;
            for (int i = 0; i < range; i++)
            {
                Destroy(agents[i].gameObject);
            }
            agents.Clear();
            sim_ = new RVOSimulator();
            Start();
        }

        if (!reachedGoal())
        {
            /*Changing preferred velocity*/
            setPreferredVelocities();
            /* Doing a step*/
            doStep(false);
            /*Printing Time  * May harm performances */
            //Debug.Log(sim_.getGlobalTime());
            // This loop will change position,  facing direction of the 3D agents on the unity scene
            // They will move accordingly to the goal you gave them in the Setup
            // you can also change their position in order to create a mouvement loop
            for (int i = 0; i < getNumAgents(); ++i)
            {
                 RVO.Vector2 position = getPosition(i);
                // Determine if the agent is in the group One ...
                 if (i < getNumAgents() / 2)
                 {

                     if (position.x() >= 90.0f)
                     {
                        RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                        agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
                        setPosition(i, new RVO.Vector2(-89, position.y()));
                         agents[i].position = new Vector3(-89, 0f, position.y());
                       

                    }
                     else
                     {
                        RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                        agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
                        agents[i].position = new Vector3(position.x(), 0f, position.y());
                       
                    }
                 }
                 //... Or in the group Two
                 else
                 {
                     if (position.x() <= -90.0f)
                     {
                         
                        setPosition(i, new RVO.Vector2(89, position.y()));
                        RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                        agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
                        agents[i].position = new Vector3(89, 0f, position.y());
                        

                    }
                     else
                     {
                        RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                        agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
                        agents[i].position = new Vector3(position.x(), 0f, position.y());
                        
                        
                    }
                }
               


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

    public override void setupScenario()
    {
        /* Specify the global time step of the simulation. */
        sim_.setTimeStep(0.125f);

        /*
         * Specify the default parameters for agents that are subsequently
         * added.
         */
        sim_.setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 0.35f, 0.5f, new RVO.Vector2(0.0f, 0.0f));
        //sim_.kdTree_.buildAgentTree(true);
        /*
         * Add agents, specifying their start position, and store their
         * goals on the opposite side of the environment.
         */
         int agent_number = 0;
        if (group_b)
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    sim_.addAgent(new RVO.Vector2(-20.0f + sim_.getDefaultRadius() * 2f * i, 3 + j * sim_.getDefaultRadius() * 3f), 0, false, avoidance_b);
                    sim_.setAgentGoal(agent_number++, new RVO.Vector2(100.0f, j * sim_.getDefaultRadius() * 2));


                }
            }
        }
        else
        {
            for (int i = 0; i < 1; ++i)
            {
                for (int j = 0; j < 1; ++j)
                {
                    sim_.addAgent(new RVO.Vector2(-20.0f + sim_.getDefaultRadius() * 2f * i, 4 + j * sim_.getDefaultRadius() * 2f), 0, false, avoidance_b);
                    sim_.setAgentGoal(agent_number++, new RVO.Vector2(100.0f, j * sim_.getDefaultRadius() * 2));


                }
            }
        }
     
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j <3; ++j)
            {
                sim_.addAgent(new RVO.Vector2(20.0f + sim_.getDefaultRadius() * 2f * i,(3+ j * sim_.getDefaultRadius() * 3f )), 0, false, avoidance_b);
                sim_.setAgentGoal(agent_number++, new RVO.Vector2(-100.0f, j * sim_.getDefaultRadius() * 2));

            }
        }


         IList<RVO.Vector2> obstacle1 = new List<RVO.Vector2>();
         obstacle1.Add(new RVO.Vector2(-150f, 40f));
         obstacle1.Add(new RVO.Vector2(150f, 40f));
         obstacle1.Add(new RVO.Vector2(150f, 41f));
         obstacle1.Add(new RVO.Vector2(-150f, 41f));
         sim_.addObstacle(obstacle1);

         IList<RVO.Vector2> obstacle2 = new List<RVO.Vector2>();
        obstacle2.Add(new RVO.Vector2(-150f, -40f));
        obstacle2.Add(new RVO.Vector2(150f, -40f));
        obstacle2.Add(new RVO.Vector2(150f, -39f));
        obstacle2.Add(new RVO.Vector2(-150f, -39f));
        sim_.addObstacle(obstacle2);

        sim_.processObstacles();
        sim_.kdTree_.buildAgentTree(false);

    }
}