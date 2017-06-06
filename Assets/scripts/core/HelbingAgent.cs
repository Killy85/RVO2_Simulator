using System;
using UnityEngine;

namespace RVO
{
    internal class HelbingAgent : Agent
    {
        float lambda_, A_, B_, tau_, m_;
        float k_;
        int type_;
        Vector2 motion_dir_;

        public HelbingAgent(RVOSimulator sim, int type):base(sim){
            lambda_=5;
            A_=25.0f;
            B_=0.13f;
            tau_=0.2f;
            m_=80.0f;
            k_= 120000.0f;
            type_ = type;
                //,A_(0.5),B_(1.65),tau_(10),lambda_(5)
        
                // laure : A = 0.5, B = 1.65, tau = 2, lambda = 0.12
        }

        public override void computeNewVelocity()
        {
            acceleration_ = new Vector2(0, 0);

            // Compute goal's force
            Vector2 toto = Vector2.normalize(goal_ - position_);
            Vector2 toto2 = Vector2.normalize(goal_ - position_) - velocity_;
            Vector2 goal_force = (maxSpeed_ * Vector2.normalize(goal_ - position_) - velocity_) / tau_;
            acceleration_ = goal_force;

            // Direction of motion
            if (velocity_ != new Vector2(0, 0))
                motion_dir_ = Vector2.normalize(velocity_);
            else
                motion_dir_ = Vector2.normalize(goal_ - position_);

            // Compute obstacles' forces
            for (int i = 0; i < obstacleNeighbors_.Count; ++i)
            {
                interactWith(obstacleNeighbors_[i].Value);
            }

            // Compute neighbors' forces
            interactWithAgents();

            // Limit acceleration
            if (Vector2.abs(acceleration_) > 5)
                acceleration_ = 5 * Vector2.normalize(acceleration_);

            newVelocity_ = velocity_ + acceleration_ * sim_.getTimeStep();

            if (Vector2.abs(newVelocity_) > maxSpeed_)
                newVelocity_ = Vector2.normalize(newVelocity_) * maxSpeed_;
        }

        public override void interactWith(SuperAgent agent)
        {

        Vector2 relative_position = agent.position_ - agent.position_;
        // w_phi enables to deal with elliptical forces
        float w_phi = 1;
        float test = ((HelbingAgent)(agent)).motion_dir_ * Vector2.normalize(-relative_position);
        float test2 = ((1 + motion_dir_ * Vector2.normalize(-relative_position)) / 2);
        float A = A_;
		if(id_==0)
			A = A_*5;
            if (type_ == 1)
                //w_phi = lambda_ + (1 - lambda_)*((1+motion_dir_*normalize(-relative_position))/2);
                w_phi =(float) Math.Pow(test + 1, lambda_)/(float) Math.Pow(2f, lambda_);
        // Compute force
        float radius_m_dist = radius_ + agent.radius_ - Vector2.abs(relative_position);
        float exp_term = (radius_m_dist) / B_;
        Vector2 force = w_phi * A * (float)Math.Exp(exp_term) * Vector2.normalize(relative_position);
		// Body force
		if(radius_m_dist > 0)
			force += k_* radius_m_dist * Vector2.normalize(relative_position);
        acceleration_ += force;
	}

        public override void interactWith(Obstacle obstacle)
        {
        // Get vertex
         Obstacle obstacle2 = obstacle.next_;
        // Distance to the vertex
        Vector2 vector2Vertex = Vector2.vectorToSegment(obstacle.point_, obstacle2.point_, position_);
        // w_phi enables to deal with elliptical forces
        float w_phi = 1;
        float test = motion_dir_ * Vector2.normalize(vector2Vertex);
        if (type_ == 1)
            //w_phi = lambda_ + (1 - lambda_)*(1+motion_dir_*normalize(vector2Vertex)/2);
            w_phi =(float)( Math.Pow(test + 1, lambda_) / Math.Pow(2, lambda_));
        // Compute force
        float exp_term = (radius_ - Vector2.abs(vector2Vertex)) / B_;
        Vector2 force = w_phi * A_ *(float) Math.Exp(exp_term) * -1 * Vector2.normalize(vector2Vertex) / m_;
        // Body force
        if (Vector2.abs(vector2Vertex) < radius_)
            force += k_ * (radius_ - Vector2.abs(vector2Vertex)) * -1 * Vector2.normalize(vector2Vertex) / m_;
        acceleration_ += force;
    }

       
    }	
}