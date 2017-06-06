/*
 * This class purpose is to create a unidirectionnal corridor instance with
 * a bottleneck at the end.
 * 
 * It aim at revealing the behaviour of agents in this kind of situtation
*/
using RVO;
using System;
using System.Collections.Generic;
using UnityEngine;

class BottleNeck : Scenario
{
    public Transform agent; //List of Unity Agent(Transform)
    public Transform wall; //List of Unity Wall (Transform)
    GameObject mainCam; // Unity Scene Camera. Usefull to change view of the scene depending on the shape and size of your plane
    IList<Color> colors = new List<Color>(); // List of color used to show the speed of agents.

    /**
     * <summary>Create this scenario</summary>
     */
    public BottleNeck() : base() {}



    /**
     * <summary>Unity function. Called once at the beginning of the scenario. Used to initialize everything you need</summary>
     */
    void Start()
    {
        // Change the maximum framerate of the simulation
        Application.targetFrameRate = 60;
        //Initialisation of the list
        agents = new List<Transform>();
        // Set up the scenario. 
        setupScenario();

        //Creation of 2 walls on both side of the corridor
        Instantiate(wall, new Vector3(-25, 12.5f, 35.5f), Quaternion.identity);
        Instantiate(wall, new Vector3(-25, 12.5f, -35.5f), Quaternion.identity);

        //For each agents added in setupScenario() , add a Unity Agent at the same position
        for (int i = 0; i < getNumAgents(); ++i)
        {
            RVO.Vector2 position = getPosition(i);
            agents.Add(Instantiate(agent, new Vector3(position.x(), sim_.getAgentRadius(i)/2, position.y()), Quaternion.identity));
            agents[i].localScale = new Vector3(sim_.getAgentRadius(i), sim_.getAgentRadius(i), sim_.getAgentRadius(i));
        }
        //Filling the colors table
        colors.Add(new Color(0.000f, 0.000f, 0.804f));
        colors.Add(new Color(0.118f, 0.565f, 1.000f));
        colors.Add(new Color(0.251f, 0.878f, 0.816f));
        colors.Add(new Color(0.400f, 0.804f, 0.667f));
        colors.Add(new Color(0.604f, 0.804f, 0.196f));
    }

    /**
     * <summary>Unity function. Called once at the beginning of each frame. Used to perform the simulation</summary>
     */
    void Update()
    {
        if (!reachedGoal())
        {
            // Set the preffered velocities of agents in order to assume they are in the good direction
            setPreferredVelocities();
            doStep();
            for (int i = 0; i < getNumAgents(); ++i)
            {
                RVO.Vector2 position = getPosition(i);
                // This condition ensure that agents stay at a limited speed in the end of the corridor
                if (position.x() >= 80.0f)
                {
                    if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) > sim_.getAgentMaxSpeed() * 3 / 5)
                        setVelocity(i, sim_.getAgentVelocity(i) / 2);
                    position = getPosition(i);
                    agents[i].position = new Vector3(position.x(), 0f, position.y());


                }
                //This one is to replace agents at the beginning of the corridor if they are at the end
                if (position.x() >= 90.0f)
                {
                    setPosition(i, new RVO.Vector2(-130, position.y()));
                    position = getPosition(i);
                    agents[i].position = new Vector3(position.x(), sim_.getAgentRadius(i)/2, position.y());
                    RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));

                }
                else // Else Juste perform the simulation and change the position if the unity agent to be the same at the RVOAgent
                {
                    agents[i].position = new Vector3(position.x(), sim_.getAgentRadius(i)/2, position.y());
                    RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
                }
                setColor(i);
            }

        }
        else //If every agent reach their goals, block everybody
        {    // Might want to do
            for (int i = 0; i < getNumAgents(); ++i)
            {
                agents[i].GetComponent<Rigidbody>().isKinematic = true;
            }
        }

    }

    public override void setupScenario()
    {
        /* Specify the global time step of the simulation. */
        sim_.setTimeStep(0.25f);

        /*
         * Specify the default parameters for agents that are subsequently
         * added.
         */
        sim_.setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f, new RVO.Vector2(0.0f, 0.0f));
        //sim_.kdTree_.buildAgentTree(true);
        /*
         * Add agents, specifying their start position, and store their
         * goals on the opposite side of the environment.
         */
        for (int i = 0; i < 12; ++i)
        {
            for (int j = 0; j < 8; ++j)
            {
                sim_.addAgent(new RVO.Vector2(-80.0f + 5 * i, -30 + j * 3), 0, true, true, 15.0f, 10, 5.0f, 5.0f, 5, 2.0f, new RVO.Vector2(0, 0));
                goals.Add(new RVO.Vector2(100.0f, 0f));

            }
        }

        //Creating a wall. The obstacle vertices have to be specified in the counter clock
        //order
        IList<RVO.Vector2> obstacle1 = new List<RVO.Vector2>();
        obstacle1.Add(new RVO.Vector2(-150f, 30f));
        obstacle1.Add(new RVO.Vector2(150f, 30f));
        obstacle1.Add(new RVO.Vector2(150f, 35f));
        obstacle1.Add(new RVO.Vector2(-150f, 35f));
        sim_.addObstacle(obstacle1);


        //Creating a wall. The obstacle vertices have to be specified in the counter clock
        //order
        IList<RVO.Vector2> obstacle2 = new List<RVO.Vector2>();
        obstacle2.Add(new RVO.Vector2(-150f, -38f));
        obstacle2.Add(new RVO.Vector2(150f, -38f));
        obstacle2.Add(new RVO.Vector2(150f, -33f));
        obstacle2.Add(new RVO.Vector2(-150f, -33f));
        sim_.addObstacle(obstacle2);


        IList<RVO.Vector2> obstacle3 = new List<RVO.Vector2>();
        obstacle3.Add(new RVO.Vector2(50f, 38f));
        obstacle3.Add(new RVO.Vector2(90f, 15f));
        obstacle3.Add(new RVO.Vector2(90f, 38f));
        sim_.addObstacle(obstacle3);

        IList<RVO.Vector2> obstacle4 = new List<RVO.Vector2>();
        obstacle4.Add(new RVO.Vector2(50f, -38f));
        obstacle4.Add(new RVO.Vector2(90f, -38f));
        obstacle4.Add(new RVO.Vector2(90f, -15f));
        sim_.addObstacle(obstacle4);

        sim_.processObstacles();
        sim_.kdTree_.buildAgentTree();

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

    /** <summary>Method to change the velocity of an agent in the simulator to reflect change
     * made in the Unity scene to avoid object destruction</summary>
     <param name="i">Id of the agent we want to update velocity</param>
     <param name="vector2"> New velocity of the agent</param>
     */
    private void setVelocity(int i, RVO.Vector2 vector2)
    {
        sim_.agents_[i].velocity_ = vector2;
    }
}