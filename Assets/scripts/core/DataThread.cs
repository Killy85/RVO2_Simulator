/*
 * DataThread.cs
 * 
 *
 * Added in order to store Data and save performance at the same time
 */

using RVO;
using System.Threading;

public class DataThread
{
    /** <summary> Simulator from which data have to be saved</summary>*/
    private RVOSimulator sim_;
    /** <summary>Define if the simulation is looped or not</summary>*/
    private bool loop_;
    /** <summary> Define if the follow behaviour is present or not</summary>*/
    private bool follow_;

    /** <summary>Create a DataThread object given necessary argument </summary>
     <param name="follow" Whether the simulation is looped or not/>
        <param name="looped" Whether the simulation is looped or not/>
        <param name="sim" The simulator from which data have to be saved />*/
    public DataThread(RVOSimulator sim, bool looped, bool follow)
    {
        sim_ = sim;
        loop_ = looped;
        follow_ = follow;

        Thread myThread = new Thread(new ThreadStart(ThreadLoop));
        myThread.Start();
    }

    public void ThreadLoop()
    {
        DataSaving.saveSimulatorData(sim_,follow_);
    }

}