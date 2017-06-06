using System;
using System.IO;

namespace RVO
{
    class DataSaving
    {


        DataSaving()
        { }


        // Save the data of each agent (position, velocity, acceleration, distance with the closer, local density and agent's leader
        internal static void saveData(RVOSimulator sim, bool looped, bool follow)
        {
            string name = "Data/n";
            string num = sim.getNumAgents().ToString();
            name += num;
            if (follow)
                name += "f";
            name += "_agents_data.csv";
            System.IO.Directory.CreateDirectory("Data/");
            if (!File.Exists(name))
            {
                File.Create(name).Dispose();
                using (TextWriter tw = new StreamWriter(name))
                {
                    tw.WriteLine("Agent \t Position X \t  Position Y \t Velocity X \t Velocity Y \t  Acceleration X + \t  Acceleration Y + \t Closer Agent Distance  \t LocalDensity 2  \t  LocalDensity 9  \t AgentLeaderNo \t Heure");
                }
            }


            for (int i = 0; i < sim.getNumAgents(); i++)
            {
                using (TextWriter tw = new StreamWriter(name, true))
                {
                    tw.WriteLine(i + "\t" + sim.getAgentPosition(i).x() + "\t" + sim.getAgentPosition(i).y() + "\t" + sim.getAgentVelocity(i).x() + "\t" + sim.getAgentVelocity(i).y() + "\t" + sim.getAgentAcceleration(i).x() + "\t" + sim.getAgentAcceleration(i).y() + "\t"
                    + sim.getAgentDistanceWithCloserAgent(i, looped) + '\t'
                    + sim.getAgentLocalDensity(i, looped, 2) + '\t' + sim.getAgentLocalDensity(i, looped, 9) + '\t'
                    + sim.getAgentLeaderNo(i) + "\t" + DateTime.Now);
                }

            }


        }

        // Save the agent's data when it attempts the end of the corridor
        //Not used
        static void saveAgentData(RVOSimulator sim, int agentNo, bool follow)
        {
            string name = "Data/n";
            string num = sim.getNumAgents().ToString();
            name += num;
            if (follow)
                name += "f";
            name += "_end_data.csv";


            System.IO.File.WriteAllText(@name, agentNo.ToString() + '\t' + sim.getAgentCptDist(agentNo).ToString() + '\t' + sim.getAgentCptTime(agentNo));


        }


        public static void saveSimulatorData(RVOSimulator sim, bool follow)
        {
            string name = "Agents_moy_data.csv";

            
            if (!File.Exists(name))
            {
                string tmp= " ";
                foreach (Agent a in sim.agents_)
                {
                    tmp += a.id_ + " : vitesse_moy \t";
                }
                File.Create(name).Dispose();
                using (TextWriter tw = new StreamWriter(name))
                {
                    tw.WriteLine(tmp);
                }
            }



            using (TextWriter tw = new StreamWriter(name, true))
            {
                string tmp = " ";
                foreach(Agent a in sim.agents_)
                {
                    tmp +=" " + Vector2.abs(a.velocity_) + "\t";
                }
                tw.WriteLine(tmp);
            }


        }

    }

}

