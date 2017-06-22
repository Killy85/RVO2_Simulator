using RVO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

class TestScriptTuto : Scenario
{
    public Transform agent;
   

    public override void setupScenario()
    {
        int cmpt = 0;
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                sim_.addAgent(new RVO.Vector2(20f + i * 2.0f, 25.0f + j * 2.0f), 0, false, true, 15.0f, 10, 5.0f, 5.0f, 1.35f, 2.0f, new RVO.Vector2(0, 0));
                sim_.setAgentGoal(cmpt++, new RVO.Vector2(-25, -25));


                sim_.addAgent(new RVO.Vector2(-20f - i * 2.0f, 25.0f + j * 2.0f), 0, false, true, 15.0f, 10, 5.0f, 5.0f, 1.35f, 2.0f, new RVO.Vector2(0, 0));
                sim_.setAgentGoal(cmpt++, new RVO.Vector2(25, -25));


                sim_.addAgent(new RVO.Vector2(20f + i * 2.0f, -25.0f - j *2.0f), 0, false, true, 15.0f, 10, 5.0f, 5.0f, 1.35f, 2.0f, new RVO.Vector2(0, 0));
                sim_.setAgentGoal(cmpt++, new RVO.Vector2(-25, 25));

                sim_.addAgent(new RVO.Vector2(-20f - i * 2.0f, -25.0f - j * 2.0f), 0, false, true, 15.0f, 10, 5.0f, 5.0f, 1.35f, 2.0f, new RVO.Vector2(0, 0));
                sim_.setAgentGoal(cmpt++, new RVO.Vector2(25, 25));

            }
        }

    }


// Use this for initialization
void Start()
{
    agents = new List<Transform>(); //Initializing the list of Unity Agents
    setupScenario(); //Calling setupScenario
    this.gameObject.transform.localScale = new Vector3(7, 1, 7);// The script is attached to a plane, so this will change the size of the plane
    this.gameObject.transform.position = new Vector3(0, 0, 0); //This will change the position of this plane
    Application.targetFrameRate = 30; // This will fix the maximum framerate of the scene

    for (int i = 0; i < getNumAgents(); i++) // loop -> For each agent
    {
        RVO.Vector2 position = getPosition(i); // Getting the position of the agent in the simulator
        addAgent(agent, new Vector3(position.x(), 1.5f, position.y())); //Instantiating  the Unity Agent at the correct position
        agents[i].localScale = new Vector3(sim_.getAgentRadius(i), sim_.getAgentRadius(i), sim_.getAgentRadius(i)); //Adapting the size of the agent to the radius we set in the simulator

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
                agents[i].position = new Vector3(position.x(), 0f, position.y());
                /*  RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));*/
                //setColor(i); To go further
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
