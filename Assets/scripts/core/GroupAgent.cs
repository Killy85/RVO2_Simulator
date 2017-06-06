using System;
using System.Collections.Generic;
using UnityEngine;

namespace RVO{


    

public class GroupAgent {
        public float RVO_EPSILON = 0.00001f;
        internal IList<Agent> agents_ = new List<Agent>();
        RVOSimulator sim_;



        public GroupAgent(RVOSimulator sim)
        {
            sim_ = sim;
        }

        internal SuperAgent RepresentGroup(Agent observer){
        // Compute the left & right tangent points obtained for each agent of the group
        // First element of the pair = right tangent
        // Second element of the pair = left tangent
        IList <Pair< Vector2,Vector2> > tangents = new List<Pair<Vector2, Vector2>>() ;
        IList<Pair<Vector2,Vector2> > radii = new List<Pair<Vector2, Vector2>>();
		for(int i=0; i < agents_.Count; i++){
			tangents.Add(computeTangentsPoints(observer, agents_[i]));
			Pair<Vector2,Vector2> rads = new Pair<Vector2, Vector2>();
			rads.First = tangents[i].First - observer.position_;
			rads.Second = tangents[i].Second - observer.position_;
			radii.Add(rads);
		}
		// Compute the group tangent points (extrema)
		int rightExtremumId = 0;
		int leftExtremumId = 0;
		for(int i=1; i < tangents.Count; i++){
			// Comparison
			if(Vector2.isOnTheLeftSide(radii[rightExtremumId].First,radii[i].First))
				rightExtremumId = i;
			if(Vector2.isOnTheLeftSide(radii[i].Second,radii[leftExtremumId].Second))
				leftExtremumId = i;
		}
		// If the tangent are taking more than 180°, cannot be considered as a group
		for(int i=0; i < agents_.Count; i++){
			if (Vector2.isOnTheLeftSide(radii[rightExtremumId].First,radii[i].First)){
				//std::cout << "Problem representing groups : tangent angle > 180°\n";
				return new SuperAgent(null);
			}
			if (Vector2.isOnTheLeftSide(radii[i].Second,radii[leftExtremumId].Second)){
				//std::cout << "Problem representing groups : tangent angle > 180°\n";
				return new SuperAgent(null);
			}
		}
		// Compute bisector	
		Vector2 rightTangent = Vector2.rotation(radii[rightExtremumId].First, (float)Math.PI/2);
		Vector2 leftTangent = Vector2.rotation(radii[leftExtremumId].Second, (float)-Math.PI / 2);
		Vector2 intersectionPoint = Vector2.intersectOf2Lines(tangents[rightExtremumId].First, tangents[rightExtremumId].First + rightTangent, 
			tangents[leftExtremumId].Second, tangents[leftExtremumId].Second + leftTangent);
		// alpha/2 is more usefull than alpha
		float alpha2 = Vector2.angle(Vector2.rotation(tangents[leftExtremumId].Second - intersectionPoint,-Vector2.angle(tangents[rightExtremumId].First - intersectionPoint))) / 2;
		if(alpha2 < 0){
			Debug.Log("Problem representing groups : angle computation\n SHALL NOT HAPPEN !!! \n"); 
			// But if radii are different or if
			return new SuperAgent(null);
		}
		Vector2 bisector_normalize_vector = Vector2.normalize(observer.position_ - intersectionPoint);
		// Compute circle
		// The distance between the observer and the circle (along the bisector axis)
		float d = Single.PositiveInfinity;
		float a,b,c,delta,x;
		int constrainingAgent = 0;
		for(int i=0; i < agents_.Count; i++){
			Vector2 ic1 = agents_[i].position_ - intersectionPoint;
			a = 1 -RVOMath.sqr((float)Math.Sin(alpha2));
			b = 2 * (agents_[i].radius_ * (float)Math.Sin(alpha2) -ic1 * bisector_normalize_vector);
			c = Vector2.absSq(ic1) -RVOMath.sqr(agents_[i].radius_);
			delta = RVOMath.sqr(b) - 4 * a * c; 
			if(delta <= 0){
				if(delta < - 4 * RVO_EPSILON * c){
					return new SuperAgent(null);
				}
				else
					delta = 0;
			}
			x = (-b+(float)Math.Sqrt(delta))/(2*a);
			if(x < d){
				d = x;
				constrainingAgent = i;
			}
		}
		if (d < Vector2.abs(observer.position_ - intersectionPoint) + observer.radius_ + d*Math.Sin(alpha2)){
			return new SuperAgent(null);
		}
		SuperAgent toReturn = new SuperAgent(sim_);
		toReturn.position_ = intersectionPoint + bisector_normalize_vector * d;
		toReturn.radius_ =  d * (float) Math.Sin(alpha2);
		toReturn.velocity_ = agents_[constrainingAgent].velocity_;
		return toReturn;
	}


	internal Pair<Vector2, Vector2> computeTangentsPoints(Agent observer, Agent agent){
		// First element of the pair = left tangent
		// Second element of the pair = right tangent
		Pair<Vector2, Vector2> toReturn = new Pair<Vector2, Vector2>();
		Vector2 centers = agent.position_ - observer.position_;
		Vector2 r1a = Vector2.normalize(Vector2.rotation(centers, (float)-Math.PI/2)) * observer.radius_;
		Vector2 r1b = Vector2.normalize(Vector2.rotation(centers, (float)Math.PI/2)) * observer.radius_;
		// Compute intersection points between radius and circle
		// Right one
		Vector2 h1a = observer.position_ + r1a;
		// Left one
		Vector2 h1b = observer.position_ + r1b;
		// If the radius is the same, tangents points are perpendicular to centers vector
		if (Math.Abs(observer.radius_ - agent.radius_ ) < RVO_EPSILON){
			toReturn.First = h1a;
			toReturn.Second = h1b;
		}
		else{
			Vector2 r2a = Vector2.normalize(Vector2.rotation(centers, (float)-Math.PI/2)) * agent.radius_;
			Vector2 r2b = Vector2.normalize(Vector2.rotation(centers, (float)Math.PI/2)) * agent.radius_;
			Vector2 h2a = agent.position_ + r2a;
			Vector2 h2b = agent.position_ + r2b;
			// If tangents are parallel, radius are the same, i.e. there is no intersection point.
			if(Math.Abs(Vector2.det(h1a-h2a,h1b-h2b)) < RVO_EPSILON){
				Console.Write("Problem while computing tangent points\n SHALL NOT HAPPEN !!! \n");
				toReturn.First = h1a;
				toReturn.Second = h1b;
			}
			else{
				Vector2 intersectionPoint = Vector2.intersectOf2Lines(h1a, h2a, h1b, h2b);
				// Equivalent to :
				Vector2 circleCenter = (intersectionPoint + observer.position_) / 2;
				toReturn = Vector2.intersectOf2Circles(circleCenter, Vector2.abs(circleCenter - observer.position_), observer.position_, observer.radius_);
				// Test angles to know which one is right &  which one is left
				if(Vector2.isOnTheLeftSide(toReturn.First -observer.position_,centers)){
					Vector2 temp = toReturn.First;
					toReturn.First = toReturn.Second;
					toReturn.Second = temp;
				}
			}
		}
		return toReturn;

	}


       internal SuperAgent representGroup(Agent observer)
        {
            // Compute the left & right tangent points obtained for each agent of the group
            // First element of the pair = right tangent
            // Second element of the pair = left tangent
            IList<Pair<Vector2, Vector2>> tangents = new List<Pair<Vector2, Vector2>>();
            IList<Pair<Vector2, Vector2>> radii = new List<Pair<Vector2, Vector2>>();

            for (int i = 0; i < agents_.Count; i++)
            {
                tangents.Add(computeTangentsPoints(observer, agents_[i]));
                Pair<Vector2, Vector2> rads = new Pair<Vector2, Vector2>();
                rads.First = tangents[i].First - observer.position_;
                rads.Second = tangents[i].Second - observer.position_;
                radii.Add(rads);
            }
            // Compute the group tangent points (extrema)
            int rightExtremumId = 0;
            int leftExtremumId = 0;
            for (int i = 1; i < tangents.Count; i++)
            {
                // Comparison
                if (Vector2.isOnTheLeftSide(radii[rightExtremumId].First, radii[i].First))
                    rightExtremumId = i;
                if (Vector2.isOnTheLeftSide(radii[i].Second, radii[leftExtremumId].Second))
                    leftExtremumId = i;
            }
            // If the tangent are taking more than 180°, cannot be considered as a group
            for (int i = 0; i < agents_.Count; i++)
            {
                if (Vector2.isOnTheLeftSide(radii[rightExtremumId].First, radii[i].First))
                {
                    //std::cout << "Problem representing groups : tangent angle > 180°\n";
                    return new SuperAgent(null);
                }
                if (Vector2.isOnTheLeftSide(radii[i].Second, radii[leftExtremumId].Second))
                {
                    //std::cout << "Problem representing groups : tangent angle > 180°\n";
                    return new SuperAgent(null);
                }
            }
            // Compute bisector	
            Vector2 rightTangent = Vector2.rotation(radii[rightExtremumId].First, (float)Math.PI / 2);
            Vector2 leftTangent = Vector2.rotation(radii[leftExtremumId].Second, -(float)Math.PI / 2);
            Vector2 intersectionPoint = Vector2.intersectOf2Lines(tangents[rightExtremumId].First, tangents[rightExtremumId].First + rightTangent,
                tangents[leftExtremumId].Second, tangents[leftExtremumId].Second + leftTangent);
            // alpha/2 is more usefull than alpha
            float alpha2 = Vector2.angle(Vector2.rotation(tangents[leftExtremumId].Second - intersectionPoint, -Vector2.angle(tangents[rightExtremumId].First - intersectionPoint))) / 2;
            if (alpha2 < 0)
            {
                //std::cout << "Problem representing groups : angle computation\n SHALL NOT HAPPEN !!! \n"; 
                // But if radii are different or if
                return new SuperAgent(null);
            }
            Vector2 bisector_normalize_vector = Vector2.normalize(observer.position_ - intersectionPoint);
            // Compute circle
            // The distance between the observer and the circle (along the bisector axis)
            float d = Single.PositiveInfinity;
            float a, b, c, delta, x;
            int constrainingAgent = 0;
            for (int i = 0; i < agents_.Count; i++)
            {
                Vector2 ic1 = agents_[i].position_ - intersectionPoint;
                a = 1 - Vector2.sqr((float)Math.Sin(alpha2));
                b = 2 * (agents_[i].radius_ * (float)Math.Sin(alpha2) - ic1 * bisector_normalize_vector);
                c = Vector2.absSq(ic1) - Vector2.sqr(agents_[i].radius_);
                delta = Vector2.sqr(b) - 4 * a * c;
                if (delta <= 0)
                {
                    if (delta < -4 * RVO_EPSILON * c)
                    {
                        return new SuperAgent(null);
                    }
                    else
                        delta = 0;
                }
                x = (-b + (float)Math.Sqrt(delta)) / (2 * a);
                if (x < d)
                {
                    d = x;
                    constrainingAgent = i;
                }
            }
            if (d < Vector2.abs(observer.position_ - intersectionPoint) + observer.radius_ + d * Math.Sin(alpha2))
            {
                return new SuperAgent(null);
            }
            SuperAgent toReturn = new SuperAgent(sim_);
            toReturn.position_ = intersectionPoint + bisector_normalize_vector * d;
            toReturn.radius_ = d *(float) Math.Sin(alpha2);
            toReturn.velocity_ = agents_[constrainingAgent].velocity_;
            return toReturn;

        }


        internal IList<Agent> getAgents()
        {
            return this.agents_;
        }
    }

}