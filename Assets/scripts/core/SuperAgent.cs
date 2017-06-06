namespace RVO
{
    public class SuperAgent {

            internal float radius_;
            internal Vector2 position_;
            internal Vector2 velocity_;
            internal RVOSimulator sim_;

        internal SuperAgent(RVOSimulator sim) {
            sim_ = sim;
            position_ = new Vector2();
            radius_ = 1.0f;
            velocity_= new Vector2();
        }

    }
}