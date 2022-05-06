#ifndef FETCHMOVEMENT_H
#define FETCHMOVEMENT_H

#include <cmath>
#include <iostream>
#include <fstream>
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"

#define V_MAX 0.3
#define V_MIN 0.05
#define W_MAX 0.5
#define G_MAX (V_MAX * W_MAX)
#define TARGET_DIST 1  // was 0.75
#define CLOSEST_DIST (TARGET_DIST - 0.05)
#define FORWARD_ANGULAR_CHANGE 2*M_PI/180
#define REVERSE_ANGULAR_CHANGE 5*M_PI/180

/*!
 * \brief The FetchMovement class
 * \details
 * \author Adam Scicluna
 * \author Tor Wei Lim
 * \author Nikhil Senthilvel
 * \date 6/05/2022
 */

class FetchMovement
{
public:
    // Constructor
    FetchMovement();
    // Destructor
    ~FetchMovement();


    /*! @brief Calculates an optimal angular and linear velocity for the Fetch to travel,
     *  based on a calculated curvature where the range to the next goal point is the chosen
     *  'look-ahead point'. Angular and Linear Velocity are proportional to this path curvature:
     *  w = curvature*V
     *
     *  @param[in] range - The current distance from the fetch robot to its goal pose.
     *  @param[in] bearing - The bearing difference from the Fetch heading to goal position.
     *  @param[in] lin - Reference to the variable controlling linear velocity of Fetch.
     *  @param[in] ang - Reference to the variable controlling angular velocity of Fetch.
     *  @note The lin and ang parameters are updated via reference, meaning no return
     *  variable is necessary. For optimisation, if the bearing difference is between
     *  +- 2 degrees when the Fetch is moving forward, angular velocity will be set to
     *  zero. Similarly, when reversing, the angular velocity will be set to zero if the
     *  bearing difference is within +- 5 degrees, a larger range to allow for the Fetch
     *  to reverse more quickly as the target distance decreases.
     */
    void purePursuit2(double range, double x, double bearing, double &lin, double &ang, double &state);


    /*! @brief Calculates the Roll, Pitch and Yaw via reference of a Robot given its pose 
     *  described as a a Quaternion being the input parameter.
     *
     *  @param[in] fetchRoll - Reference to the variable holding the current roll value of the Fetch.
     *  @param[in] fetchPitch - Reference to the variable holding the current pitch value of the Fetch.
     *  @param[in] fetchYaw - Reference to the variable holding the current yaw value of the Fetch.
     *  @note The roll, pitch and yaw parameters are updated via reference, meaning no return
     *  variable is necessary. The function obtains the roll, pitch, yaw from the pose Quaternion
     *  by first converting the Quaternion to a 3x3 Rotation Matrix, in which the Roll, Pitch and Yaw
     *  can then be directly extracted.
     */
    void rpyFromPose(geometry_msgs::PoseStamped pose, double &fetchRoll, double &fetchPitch, double &fetchYaw);

    /*! @brief Converts an angle outside of the range -2PI to 2PI radians to the equivalent angle
     *  inside that range.
     *
     *  @param[in] theta - The angle to be mapped inside the range -2PI to 2PI radians.
     *
     *  @return double - Function will return the input angle in radians mapped to a value in the
     *  range -2PI to 2PI.
     */
    double normaliseAngle(double theta);

private:
    std::ofstream myfile;
};

#endif // FETCHMOVEMENT_H
