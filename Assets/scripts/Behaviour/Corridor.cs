using RVO;
using System;
using System.Collections.Generic;
using UnityEngine;

class Corridor : Scenario
{
    //Declaration of the scenario

    public Transform agent;
    public Transform wall;
    IList<Color> colors = new List<Color>();


    public Corridor() : base() { }



    public override void setupScenario()
    {
        /* Specify the global time step of the simulation. */
        sim_.setTimeStep(0.125f);

        /*
         * Specify the default parameters for agents that are subsequently
         * added.
         */
        sim_.setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f, new RVO.Vector2(0.0f, 0.0f));
        //sim_.kdTree_.buildAgentTree(true);
        /*
         * Add agents, specifying their start position, and store their
         * goals.
         * In that case, every agent have the same goal which is the end of the corridor.
         * the scene script might respawn agent when when cross a certain position
         * This work has to be done in 
         */
        int cmpt = 0;
        for (int i = 0; i <60; ++i)
        {
            for (int j = 0; j < 12; ++j)
            {
                sim_.addAgent(new RVO.Vector2(-50.0f + i, -25 + j *3), 0, true, false,15.0f,10,5.0f,5.0f,1.35f,2.0f, new RVO.Vector2(0,0));
                sim_.setAgentGoal(cmpt++,new RVO.Vector2(100.0f, 0f));

            }
        }




        //Adding a wall
        IList<RVO.Vector2> obstacle1 = new List<RVO.Vector2>();
        obstacle1.Add(new RVO.Vector2(-150f, 30f));
        obstacle1.Add(new RVO.Vector2(150f, 30f));
        obstacle1.Add(new RVO.Vector2(150f, 35f));
        obstacle1.Add(new RVO.Vector2(-150f, 35f));
        sim_.addObstacle(obstacle1);

        //Adding a wall
        IList<RVO.Vector2> obstacle2 = new List<RVO.Vector2>();
        obstacle2.Add(new RVO.Vector2(-150f, -38f));
        obstacle2.Add(new RVO.Vector2(150f, -38f));
        obstacle2.Add(new RVO.Vector2(150f, -33f));
        obstacle2.Add(new RVO.Vector2(-150f, -33f));
        sim_.addObstacle(obstacle2);


        IList<RVO.Vector2> obstacle3 = new List<RVO.Vector2>();
        obstacle3.Add(new RVO.Vector2(-150f, -38f));
        obstacle3.Add(new RVO.Vector2(-145f, -38f));
        obstacle3.Add(new RVO.Vector2(-145f, 38f));
        obstacle3.Add(new RVO.Vector2(-150f, 38f));
        sim_.addObstacle(obstacle3);

        sim_.processObstacles();
        sim_.kdTree_.buildAgentTree(false);

    }

    // Use this for initialization
    void Start()
    {
        agents = new List<Transform>();
        /* Set up the scenario. */
        setupScenario();
        Instantiate(wall, new Vector3(-25, 12.5f, 35.5f), Quaternion.identity);
        Instantiate(wall, new Vector3(-25, 12.5f, -35.5f), Quaternion.identity);
        colors.Add(new Color(0.000f, 0.000f, 0.804f));
        colors.Add(new Color(0.118f, 0.565f, 1.000f));
        colors.Add(new Color(0.251f, 0.878f, 0.816f));
        colors.Add(new Color(0.400f, 0.804f, 0.667f));
        colors.Add(new Color(0.604f, 0.804f, 0.196f));

        /* Perform (and manipulate) the simulation. */
        for (int i = 0; i < getNumAgents(); i++)
        {
            RVO.Vector2 position = getPosition(i);
            addAgent(agent, new Vector3(position.x(), 1.5f, position.y()));
            agents[i].localScale = new Vector3(sim_.getAgentRadius(i), sim_.getAgentRadius(i), sim_.getAgentRadius(i));

        }

    }

    /**<summary> This methode change the color of the UntyAgent according to 
    * his current velocity compare to his maximumvelocity</summary>
    * There might be a problem with this one if the prefab didn't have the same structure. 
    * You might have to look further in the prefab structur in order to locate
    * his Meshrenderer and change the material color.
    * <param name="i" > Number of the agent the color have to be change</param>*/
    void setColor(int i)
    {
        float max_vel = sim_.getAgentMaxSpeed();
        if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= max_vel / 5)
        {
            agents[i].GetComponent<MeshRenderer>().material.color = colors[0];
        }
        else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 2 * max_vel / 5)
        {
            agents[i].GetComponent<MeshRenderer>().material.color = colors[1];
}
        else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 3 * max_vel / 5)
        {
            agents[i].GetComponent<MeshRenderer>().material.color = colors[2];
        }
        else if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) <= 4 * max_vel / 5)
        {
            agents[i].GetComponent<MeshRenderer>().material.color = colors[3];
        }
        else
        {
            agents[i].GetComponent<MeshRenderer>().material.color = colors[4];
        }
    }
    // Update is called once per frame
    void Update()
    {
        if (!reachedGoal())
        {
            setPreferredVelocities();
            doStep(false);

            /* Output the current global time. */
            //print(Simulator.Instance.getGlobalTime());
            for (int i = 0; i < getNumAgents(); ++i)
            {
                RVO.Vector2 position = getPosition(i);
                if (position.x() >= 80.0f)
                {
                    if(RVO.Vector2.abs(sim_.getAgentVelocity(i)) > sim_.getAgentMaxSpeed()*3/5)
                    setVelocity(i, sim_.getAgentVelocity(i)/2);
                    position = getPosition(i);
                    agents[i].position = new Vector3(position.x(), 0f, position.y());
                   /* RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));*/

                }
                if (position.x() >= 90.0f)
                {
                    setPosition(i, new RVO.Vector2(-140, position.y()));
                    position = getPosition(i);
                    agents[i].position = new Vector3(position.x(), 0f, position.y());
                  /*  RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));*/

                }
                else
                {
                    agents[i].position = new Vector3(position.x(),0f, position.y());
                  /*  RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));*/
                }
                setColor(i);
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

    private void setVelocity(int i, RVO.Vector2 vector2)
    {
        sim_.agents_[i].velocity_ = vector2;
    }
}
