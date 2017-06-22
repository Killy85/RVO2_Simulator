using CenterSpace.Free;
using RVO;
using System;
using System.Collections.Generic;
using Troschuetz.Random;
using UnityEngine;
using UnityEngine.UI;

class Parametrable : Scenario
{
    //Declaration of the scenario

    public Transform agent;
    public Transform wall;
    public Text numAgent;
    public Text length;
    public Text width ;
    public GameObject plane;
    public GameObject panel;
    GameObject mainCam;

    IList<Color> colors = new List<Color>();


    public Parametrable() : base() { }

    float corridor_lenght = 45;
    float corridor_width = 15f;
    float agents_number = 65;
    float ped_radius_ = 1.4f;

    public override void setupScenario()
    {

        mainCam = GameObject.Find("Main Camera");
        mainCam.transform.position = new Vector3(corridor_lenght/2,100,corridor_width/2);

        sim_.setTimeStep(0.125f);

        sim_.setAgentDefaults(15.0f, 10, 5.0f, 5.0f, ped_radius_, 1.0f, new RVO.Vector2(0.0f, 0.0f));


        plane.transform.localScale = new Vector3(corridor_lenght/10,1,corridor_width/10);
        plane.transform.position = new Vector3(corridor_lenght/2, 0, corridor_width/2);
        NormalDist normal = new NormalDist(1.2, 0.3);
        System.Random generator_ = new System.Random();


        MT19937Generator generator = new MT19937Generator();
        StandardGenerator sg = new StandardGenerator();

        ContinuousUniformDistribution x_distribution = new ContinuousUniformDistribution(generator);
        NormalDistribution y_distribution = new NormalDistribution(generator);
        y_distribution.Mu = corridor_width / 2;
        y_distribution.Sigma = sim_.getDefaultRadius();
           

            for (int i = 0; i <agents_number; ++i)
        {
            float x = (float)x_distribution.NextDouble() * corridor_lenght % corridor_lenght;
            float y = (float)((y_distribution.NextDouble() * corridor_width) - 9) % corridor_width;

            RVO.Vector2 position = new RVO.Vector2(x, y );
                sim_.addAgent(position, 0, true, true, 15.0f, 10, 5.0f, 5.0f, 1, 1.0f, new RVO.Vector2(0, 0));
                sim_.setAgentGoal(i, new RVO.Vector2(corridor_lenght * 4,y));

        }

        IList<RVO.Vector2> north_Wall = new List<RVO.Vector2>();
        north_Wall.Add(rotation(new RVO.Vector2(-100.0f, corridor_width ), 0f));
        north_Wall.Add(rotation(new RVO.Vector2(-100.0f, corridor_width  + 1), 0f));
        north_Wall.Add(rotation(new RVO.Vector2(100.0f + corridor_lenght, corridor_width  + 1), 0f));
        north_Wall.Add(rotation(new RVO.Vector2(100.0f + corridor_lenght, corridor_width ), 0f));
        sim_.addObstacle(north_Wall);

        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = new Vector3(corridor_lenght / 2, 0, corridor_width + 0.5f);
        cube.transform.localScale = new Vector3(corridor_lenght - 0, 4, 1);



        IList<RVO.Vector2> south_Wall = new List<RVO.Vector2>();
        south_Wall.Add(new RVO.Vector2(100 + corridor_lenght, 0));
        south_Wall.Add(new RVO.Vector2(100 + corridor_lenght, -1));
        south_Wall.Add(new RVO.Vector2(-100,-1));
        south_Wall.Add(new RVO.Vector2(-100, 0));
        sim_.addObstacle(south_Wall);


        List<RVO.Vector2> north_neck = new List<RVO.Vector2>();
        north_neck.Add(rotation(new RVO.Vector2(corridor_lenght - corridor_width / 2, corridor_width), 90f));
        north_neck.Add(rotation(new RVO.Vector2(corridor_lenght - corridor_width / 2, corridor_width + 1), 90f));
        north_neck.Add(rotation(new RVO.Vector2(corridor_lenght + corridor_width / 2, corridor_width + 1),90f));
        north_neck.Add(rotation(new RVO.Vector2(corridor_lenght + corridor_width / 2, corridor_width), 90f));
        sim_.addObstacle(north_neck);

        GameObject cube2 = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube2.transform.position = new Vector3(corridor_lenght / 2, 0,-0.5f);
        cube2.transform.localScale = new Vector3(corridor_lenght - 0, 4, 1);

        sim_.processObstacles();
        sim_.kdTree_.buildAgentTree(false);

    }

    public void Start()
    {
        numAgent = GameObject.Find("nbAgents").GetComponentInChildren<Text>();

        length = GameObject.Find("corLength").GetComponentInChildren<Text>();
        width = GameObject.Find("corWidth").GetComponentInChildren<Text>();

    }

    public void Start_Click()
    {
        if (numAgent.text.ToString() != "")
        {
            agents_number = Int32.Parse(numAgent.text.ToString());
            Debug.Log("Agents oui ");
        }
        else
        {
            agents_number = 40;
            Debug.Log("Agents non ");
        }
        if (width.text.ToString() != "")
        {
            corridor_width = Int32.Parse(width.text.ToString());
            Debug.Log("Width oui ");
        }
        else
        {
            corridor_width = 10;
            Debug.Log("Width non ");
        }
        if (length.text.ToString() != "")
        {
           corridor_lenght = Int32.Parse(length.text.ToString());
            Debug.Log("Length oui ");
        }
        else
        {
           corridor_lenght =30;
            Debug.Log("Length non ");
        }

        panel.SetActive(false);
        Application.targetFrameRate = 60;
        agents = new List<Transform>();
        setupScenario();


        colors.Add(new Color(0.000f, 0.000f, 0.804f));
        colors.Add(new Color(0.118f, 0.565f, 1.000f));
        colors.Add(new Color(0.251f, 0.878f, 0.816f));
        colors.Add(new Color(0.400f, 0.804f, 0.667f));
        colors.Add(new Color(0.604f, 0.804f, 0.196f));


        for (int i = 0; i < getNumAgents(); i++)
        {
            RVO.Vector2 position = getPosition(i);
            agents.Add(Instantiate(agent, new Vector3(position.x(), 1.5f, position.y()), Quaternion.identity));
            agents[i].GetComponent<MeshRenderer>().material.color = colors[0];
            agents[i].localScale = new Vector3(sim_.getAgentRadius(i), sim_.getAgentRadius(i), sim_.getAgentRadius(i));
        }

    }


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

    void Update()
    {

        if (!reachedGoal())
        {
            setPreferredVelocities();
            doStep(true);
           /* if(sim_.agents_.Count < agents_number) {
                int j = new System.Random().Next(1,3);
                int k = new System.Random().Next(1, 8);
                RVO.Vector2 position2 = new RVO.Vector2(0,k* corridor_width/8);
                sim_.addAgent(position2, 0, true, true,15.0f, 10, 5.0f, 5.0f, j, 1.0f, new RVO.Vector2(0, 0));
                goals.Add(new RVO.Vector2(corridor_lenght * 4, corridor_width / 2));
                RVO.Vector2 position = getPosition(sim_.agents_.Count - 1);
                agents.Add(Instantiate(agent, new Vector3(position.x(), sim_.getAgentRadius(agents.Count - 1)/2, position.y()), Quaternion.identity));
                agents[agents.Count-1].GetComponent<MeshRenderer>().material.color = colors[0];
                agents[agents.Count - 1].localScale = new Vector3(sim_.getAgentRadius(agents.Count - 1), sim_.getAgentRadius(agents.Count - 1), sim_.getAgentRadius(agents.Count - 1));
            }*/

            for (int i = 0; i < getNumAgents(); ++i)
            {
                RVO.Vector2 position = getPosition(i);
                if (position.x() >= corridor_lenght/8*7 )
                {
                    if (RVO.Vector2.abs(sim_.getAgentVelocity(i)) > sim_.getAgentMaxSpeed() * 3 / 5)
                        setVelocity(i, sim_.getAgentVelocity(i) * 7/8);
                    position = getPosition(i);
                    agents[i].position = new Vector3(position.x(), sim_.getAgentRadius(i)/2, position.y());
                    /* RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                     agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));*/

                }

                if (position.x() >= corridor_lenght)
                {

                    Collider[] et = Physics.OverlapSphere(new Vector3(-140, sim_.getDefaultRadius()*2.1f, position.y_), sim_.getDefaultRadius()*2);
                    if (et.Length > 0)
                    {
                        setPosition(i, new RVO.Vector2(position.x(), position.y()));
                    }
                    else
                    {
                        setPosition(i, new RVO.Vector2(0, position.y()));
                       /* sim_.addAgent(new RVO.Vector2(-140, position.y() + 1), 0, true, true);
                        goals.Add(new RVO.Vector2(100.0f, 0f));
                        agents.Add(Instantiate(agent, new Vector3(-140, 1.5f, position.y() + 1), Quaternion.identity));
                        agents[agents.Count - 1].GetComponent<MeshRenderer>().material.color = colors[0];
                        sim_.getAgent(getNumAgents() - 1).leader_ = sim_.getAgent(i);*/
                    }
                    position = getPosition(i);
                    agents[i].position = new Vector3(position.x(), sim_.getAgentRadius(i) / 2, position.y());
                    //RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    //agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));

                    

                }
                else
                {
                    
                    agents[i].position = new Vector3(position.x(), sim_.getAgentRadius(i) / 2, position.y());
                    //RVO.Vector2 vector2 = sim_.getAgentVelocity(i);
                    //agents[i].rotation = Quaternion.LookRotation(new Vector3(vector2.x_, 0, vector2.y_));
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
     RVO.Vector2 rotation( RVO.Vector2 vector, float angle)
	{
        
        float x_prime = (float)Math.Cos(angle) * vector.x() - (float)Math.Sin(angle) * vector.y();
        float y_prime = (float)Math.Sin(angle) * vector.x() + (float)Math.Cos(angle) * vector.y();
		return new RVO.Vector2(x_prime, y_prime);
}

    private void setVelocity(int i, RVO.Vector2 vector2)
    {
        sim_.agents_[i].velocity_ = vector2;
    }
}
