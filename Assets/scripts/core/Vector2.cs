/*
 * Vector2.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Globalization;

namespace RVO
{
    /**
     * <summary>Defines a two-dimensional vector.</summary>
     */
    public struct Vector2
    {
        internal float x_;
        internal float y_;

        /**
         * <summary>Constructs and initializes a two-dimensional vector from the
         * specified xy-coordinates.</summary>
         *
         * <param name="x">The x-coordinate of the two-dimensional vector.
         * </param>
         * <param name="y">The y-coordinate of the two-dimensional vector.
         * </param>
         */
        public Vector2(float x, float y)
        {
            x_ = x;
            y_ = y;
        }

        /**
         * <summary>Returns the string representation of this vector.</summary>
         *
         * <returns>The string representation of this vector.</returns>
         */
        public override string ToString()
        {
            return "(" + x_.ToString(new CultureInfo("").NumberFormat) + "," + y_.ToString(new CultureInfo("").NumberFormat) + ")";
        }

        /**
         * <summary>Returns the x-coordinate of this two-dimensional vector.
         * </summary>
         *
         * <returns>The x-coordinate of the two-dimensional vector.</returns>
         */
        public float x()
        {
            return x_;
        }

        /**
         * <summary>Returns the y-coordinate of this two-dimensional vector.
         * </summary>
         *
         * <returns>The y-coordinate of the two-dimensional vector.</returns>
         */
        public float y()
        {
            return y_;
        }

        /**
         * <summary>Computes the dot product of the two specified
         * two-dimensional vectors.</summary>
         *
         * <returns>The dot product of the two specified two-dimensional
         * vectors.</returns>
         *
         * <param name="vector1">The first two-dimensional vector.</param>
         * <param name="vector2">The second two-dimensional vector.</param>
         */
        public static float operator *(Vector2 vector1, Vector2 vector2)
        {
            return (float)Math.Round(vector1.x_ * vector2.x_ + vector1.y_ * vector2.y_,3);
        }

        /**
         * <summary>Computes the scalar multiplication of the specified
         * two-dimensional vector with the specified scalar value.</summary>
         *
         * <returns>The scalar multiplication of the specified two-dimensional
         * vector with the specified scalar value.</returns>
         *
         * <param name="scalar">The scalar value.</param>
         * <param name="vector">The two-dimensional vector.</param>
         */
        public static Vector2 operator *(float scalar, Vector2 vector)
        {
            return vector * (float)Math.Round(scalar,3);
        }

        /**
         * <summary>Computes the scalar multiplication of the specified
         * two-dimensional vector with the specified scalar value.</summary>
         *
         * <returns>The scalar multiplication of the specified two-dimensional
         * vector with the specified scalar value.</returns>
         *
         * <param name="vector">The two-dimensional vector.</param>
         * <param name="scalar">The scalar value.</param>
         */
        public static Vector2 operator *(Vector2 vector, float scalar)
        {
            return new Vector2((float)Math.Round(vector.x_ * scalar,3), (float)Math.Round(vector.y_ * scalar,3));
        }

        /**
         * <summary>Computes the scalar division of the specified
         * two-dimensional vector with the specified scalar value.</summary>
         *
         * <returns>The scalar division of the specified two-dimensional vector
         * with the specified scalar value.</returns>
         *
         * <param name="vector">The two-dimensional vector.</param>
         * <param name="scalar">The scalar value.</param>
         */
        public static Vector2 operator /(Vector2 vector, float scalar)
        {
            return new Vector2((float)Math.Round(vector.x_ / scalar,3), (float)Math.Round(vector.y_ / scalar,3));
        }

        /**
         * <summary>Computes the vector sum of the two specified two-dimensional
         * vectors.</summary>
         *
         * <returns>The vector sum of the two specified two-dimensional vectors.
         * </returns>
         *
         * <param name="vector1">The first two-dimensional vector.</param>
         * <param name="vector2">The second two-dimensional vector.</param>
         */
        public static Vector2 operator +(Vector2 vector1, Vector2 vector2)
        {
            return new Vector2((float)Math.Round(vector1.x_ + vector2.x_,3), (float)Math.Round(vector1.y_ + vector2.y_,3));
        }

        /**
         * <summary>Computes the vector difference of the two specified
         * two-dimensional vectors</summary>
         *
         * <returns>The vector difference of the two specified two-dimensional
         * vectors.</returns>
         *
         * <param name="vector1">The first two-dimensional vector.</param>
         * <param name="vector2">The second two-dimensional vector.</param>
         */
        public static Vector2 operator -(Vector2 vector1, Vector2 vector2)
        {
            return new Vector2((float)Math.Round(vector1.x_ - vector2.x_,3), (float)Math.Round(vector1.y_ - vector2.y_,3));
        }

        /**
         * <summary>Computes the negation of the specified two-dimensional
         * vector.</summary>
         *
         * <returns>The negation of the specified two-dimensional vector.
         * </returns>
         *
         * <param name="vector">The two-dimensional vector.</param>
         */
        public static Vector2 operator -(Vector2 vector)
        {
            return new Vector2((float)Math.Round(-vector.x_,3), (float)Math.Round(-vector.y_,3));
        }

        public static float abs(Vector2 vector)
        {
            return (float)Math.Sqrt(vector * vector);
        }

        /*!
        *  @brief      Computes the squared length of a specified two-dimensional
        *              vector.
        *  @param      vector          The two-dimensional vector whose squared length
        *                              is to be computed.
        *  @returns    The squared length of the two-dimensional vector.
        */
        public static float absSq(Vector2 vector)
        {
            return vector * vector;
        }

        /*!
        *  @brief      Computes the determinant of a two-dimensional square matrix with
        *              rows consisting of the specified two-dimensional vectors.
        *  @param      vector1         The top row of the two-dimensional square
        *                              matrix.
        *  @param      vector2         The bottom row of the two-dimensional square
        *                              matrix.
        *  @returns    The determinant of the two-dimensional square matrix.
        */
        public static float det(Vector2 vector1, Vector2 vector2)
        {
            return vector1.x() * vector2.y() - vector1.y() * vector2.x();
        }


        public static float dot(Vector2 vector1, Vector2 vector2)
        {
            return vector1.x() * vector2.x() + vector1.y() * vector2.y();
        }

        /*!
        *  @brief      Computes the normalization of the specified two-dimensional
        *              vector.
        *  @param      vector          The two-dimensional vector whose normalization
        *                              is to be computed.
        *  @returns    The normalization of the two-dimensional vector.
        */
        public static Vector2 normalize(Vector2 vector)
        {
            return vector / abs(vector);
        }

        /*!
        *  @brief      Computes a rotated vector of the specified two-dimensional
        *              vector with the specified angle
        *  @param      vector          The two-dimensional vector whose rotation
        *                              is to be computed.
        *  @param      double          The angle of the rotation
        *  @returns    The rotation of the two-dimensional vector.
        */
        public static Vector2 rotation(Vector2 vector, float angle)
        {
            float x_prime = (float)Math.Round(Math.Cos(angle) * vector.x() - (float)Math.Sin(angle) * vector.y(),3);
            float y_prime = (float)Math.Round(Math.Sin(angle) * vector.x() + (float)Math.Cos(angle) * vector.y(),3);
            return new Vector2(x_prime, y_prime);
        }

        /*!
        *  @brief      Computes the angle to the x axis of the specified 
        *				 two-dimensional vector
        *  @param      vector          The two-dimensional vector whose angle 
        *								 is to be computed
        *  @returns    The angle of the two-dimensional vector.
        */
        public static float angle(Vector2 vector)
        {
            /*if (vector.x() == 0)
            return (float)M_PI/2 * (-1 + 2*(vector.y()>0));
            else{*/
            return (float)Math.Atan2(vector.y(), vector.x());
            //}
        }

        public static bool operator !=(Vector2 vector, Vector2 vector2)
        {
            return vector2.x() != vector.x() || vector2.y() != vector.y();
        }

        public static bool operator ==(Vector2 vector, Vector2 vector2)
        {
            return vector2.x() == vector.x() && vector2.y() == vector.y();
        }

        /*!
        *  @brief      Computes the vector of the distance between the segment 
        *			   determined by the two two-dimensional vector and a point
        *  @param      pS          The first two-dimensional point of the segment
        *  @param      pE          The second two-dimensional point of the segment
        *  @param      point       The point the distance is to be computed
        *  @returns    The (perpendicular) vector to the segment
        */
        public static Vector2 vectorToSegment(Vector2 pS, Vector2 pE, Vector2 point)
        {
            // If both points are the same point, return the distance to one of the two points
            if (pS == pE)
                return pS - point;

            // s = segment vector, u = (Ps -> this) vector
            Vector2 s = pE - pS;
            Vector2 u = point - pS;
            // dp = dot product s by u
            float dp = s * u;

            // Check if projection out of the segment
            if (dp < 0)
                return pS - point;

            float s2 = s * s;

            if (dp > s2)
                return pE - point;

            // Vector to projection point
            Vector2 ah = (normalize(s) * (float)Math.Sqrt(s2)) / dp;

            return ah - u;
        }

        /*!
        *  @brief      Computes the intersection point of two lines
        *  @param      p1a		The first point of the first line
        *  @param      p2a		The second point of the first line
        *  @param      p1b		The first point of the second line
        *  @param      p2b		The second point of the second line
        *  @returns    The intersection point
        */
        public static Vector2 intersectOf2Lines(Vector2 p1a, Vector2 p2a, Vector2 p1b, Vector2 p2b)
        {
            Vector2 p1ap2a = p2a - p1a;
            Vector2 p1bp2b = p2b - p1b;
            Vector2 p1bp1a = p1a - p1b;
            if (Math.Abs(det(p1ap2a, p1bp2b)) < RVOMath.RVO_EPSILON)
                // Error case, parallel lines
                return new Vector2(Single.PositiveInfinity, Single.PositiveInfinity);
            else
            {
                float t = det(p1bp2b, p1bp1a) / det(p1ap2a, p1bp2b);
                return p1a + p1ap2a * t;
            }
        }

        /*!
        *  @brief      Computes the square of a float.
        *  @param      a               The float to be squared.
        *  @returns    The square of the float.
        */
        public static float sqr(float a)
        {
            return a * a;
        }

        /*!
        *  @brief      Computes the intersection point of two circles
        *  @param      c1		The center of the first circle
        *  @param      r1		The radius of the first circle
        *  @param      c2		The center of the second circle
        *  @param      r2		The radius of the second circle
        *  @returns    The intersection point
        */
        public static Pair<Vector2, Vector2> intersectOf2Circles(Vector2 c1, float r1, Vector2 c2, float r2)
        {
            Pair<Vector2, Vector2> toReturn = new Pair<Vector2, Vector2>();
            toReturn.First = new Vector2(Single.PositiveInfinity, Single.PositiveInfinity);
            toReturn.Second = new Vector2(Single.PositiveInfinity, Single.PositiveInfinity);
            if (absSq(c1 - c2) < sqr(r1 - r2) - RVOMath.RVO_EPSILON || absSq(c1 - c2) > sqr(r1 + r2) + RVOMath.RVO_EPSILON)
                // ERROR_ case, no intersection
                return toReturn;
            else
            {
                if (Math.Abs(c1.y() - c2.y()) < RVOMath.RVO_EPSILON)
                {
                    float x = (sqr(r1) - sqr(r2) - sqr(c1.x()) + sqr(c2.x())) / (2 * (c2.x() - c1.x()));

                    // a = 1
                    float b = -2 * c2.y();
                    float c = absSq(c2) - sqr(r2) + sqr(x) - 2 * x * c2.x();
                    float delta = sqr(b) - 4 * c;
                    float y1 = (float)(-b + Math.Sqrt(delta)) / 2;
                    float y2 = (float)(-b - Math.Sqrt(delta)) / 2;

                    toReturn.First = new Vector2(x, y1);
                    toReturn.Second = new Vector2(x, y2);
                }
                else
                {
                    float M = (c2.x() - c1.x()) / (c2.y() - c1.y());
                    float N = (sqr(r1) - sqr(r2) - absSq(c1) + absSq(c2)) / (2 * (c2.y() - c1.y()));

                    float a = 1 + sqr(M);
                    float b = 2 * (M * (c2.y() - N) - c2.x());
                    float c = absSq(c2) + sqr(N) - 2 * N * c2.y() - sqr(r2);

                    float delta = sqr(b) - 4 * a * c;
                    float x1 = (float)(-b + Math.Sqrt(delta)) / (2 * a);
                    float x2 = (float)(-b - Math.Sqrt(delta)) / (2 * a);
                    float y1 = N - x1 * M;
                    float y2 = N - x2 * M;

                    toReturn.First = new Vector2(x1, y1);
                    toReturn.Second = new Vector2(x2, y2);
                }
                return toReturn;
            }
        }



        /*! NOT SURE OF THIS FUNCTION
            *  @brief      Check if the current vector is on the left side of a specified vector.
            *  @param      v1          The first vector.
            *  @param      toCompare          The specified vector to be compared with.
            *  @returns    True if it is on the left side, false otherwise.
            */
        public static bool isOnTheLeftSide(Vector2 v1, Vector2 toCompare)
        {
            return angle(rotation(v1, -angle(toCompare))) > RVOMath.RVO_EPSILON;
        }

    }
}
