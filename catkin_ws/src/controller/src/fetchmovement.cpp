#include "fetchmovement.h"
#include "math.h"
#include <cmath>
#include <cstdlib>



FetchMovement::FetchMovement()
{
    myfile.open("pure_pursuit_log.txt");
}

FetchMovement::~FetchMovement()
{
    myfile.close();
}

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
void FetchMovement::purePursuit2(double range, double x, double bearing, double &lin, double &ang, double &state)
{
    bool prioAng = false;

    // Our goal look-ahead point is 1m from the target, so range-1
    double goalRange = range - TARGET_DIST;
    double goalRangeMultiplier_FORWARD = goalRange * 1.5;
    double goalRangeMultiplier_BACKWARDS = goalRange * 2;
    // By similar triangles, get goalX
    double goalX = goalRange*x/range;

    myfile << "NEW TIMESTAMP \n";
    myfile << "Closest Allowable Dist = " << CLOSEST_DIST << "\n";
    myfile << "Range = " << range << "\n";
    myfile << "Goal Range = " << goalRange << "\n";
    myfile << "x = " << x << "\n";
    myfile << "Goal X = " << goalX << "\n";
    myfile << "Bearing = " << bearing << "\n";

    // Calculating the curvature of the path, gamma
    double gamma = (2*std::abs(goalX))/std::pow(goalRange, 2);

    myfile << "Gamma = " << gamma << "\n";

    // Check Distance. If in allowable range, don't move.
    if(range <= TARGET_DIST && range > CLOSEST_DIST)
    {
      //Checks if the QR tracker has found
      if(state == 3.0)
      {
        // No movement required
        lin = 0;
        ang = 0;
        myfile << "SCENARIO 0 \n";
      }
    }

    // IF GOALRANGE >= 0 -> MOVE FORWARDS
    else if(goalRange >= 0 && state == 3.0)
    {
        //std::cout << "Goal Range > 0" << std::endl;
        myfile << "GOAL RANGE > 0 \n";

        // If the angle is relatively small, go full steam ahead
        if(-FORWARD_ANGULAR_CHANGE <= bearing && bearing <= FORWARD_ANGULAR_CHANGE)
        {
            // Set Linear Velocity to be distance to goal point - targest distance m/s
            lin = goalRangeMultiplier_FORWARD;
            if(lin > V_MAX)
            {
                lin = V_MAX;
            }
            else if(lin < V_MIN)
            {
                lin = V_MIN;
            }
            ang = 0;
            myfile << "SCENARIO 1 \n";
        }

        // If bearing error is between 0 and PI, rotate clockwise (w < 0)
        // Let the bearing error be the input angular velocity, scale linear based on gamma.
        else if(FORWARD_ANGULAR_CHANGE < bearing && bearing <= M_PI)
        {
            myfile << "SCENARIO 2 \n";

            // Prioritise Linear Velocity
            if(prioAng != true)
            {
                lin = goalRange;
                if(lin > V_MAX)
                {
                    lin = V_MAX;
                }
                else if(lin < V_MIN)
                {
                    lin = V_MIN;
                }

                // Calc Angular Velocity using Gamma Proportionality
                ang = -(gamma*lin);
                if(ang < -W_MAX)
                {
                    ang = -W_MAX;
                }
            }

            // Prioritise Angular Velocity
            else
            {
                ang = -bearing/2;
                if(ang < -W_MAX)
                {
                    ang = -W_MAX;
                }
                lin = std::abs(ang)/gamma;
                if(lin > goalRangeMultiplier_FORWARD)
                {
                    lin = goalRangeMultiplier_FORWARD;
                    if(lin > V_MAX)
                    {
                        lin = V_MAX;
                    }
                    else if(lin < V_MIN)
                    {
                        lin = V_MIN;
                    }
                }
            }

            myfile << "LIN INSIDE SCENARIO 2: " << lin << "\n";
        }

        // If bearing error is between 0 (2 degrees in this case) and -PI, rotate anticlockwise (w > 0)
        // Let the bearing error be the input angular velocity, scale linear based on gamma.
        else if(bearing < -FORWARD_ANGULAR_CHANGE && bearing >= -M_PI)
        {
            myfile << "SCENARIO 3 \n";

            // Prioritise Linear Velocity
            if(prioAng != true)
            {
                lin = goalRange;
                if(lin > V_MAX)
                {
                    lin = V_MAX;
                }
                else if(lin < V_MIN)
                {
                    lin = V_MIN;
                }

                // Calc Angular Velocity using Gamma Proportionality
                ang = (gamma*lin);
                if(ang > W_MAX)
                {
                    ang = W_MAX;
                }
            }

            // Prioritise Angular Velocity
            else
            {
                ang = -bearing/2;
                if(ang > W_MAX)
                {
                    ang = W_MAX;
                }
                lin = std::abs(ang)/gamma;
                if(lin > goalRangeMultiplier_FORWARD)
                {
                    lin = goalRangeMultiplier_FORWARD;
                    if(lin > V_MAX)
                    {
                        lin = V_MAX;
                    }
                    else if(lin < V_MIN)
                    {
                        lin = V_MIN;
                    }
                }
            }

            myfile << "LIN INSIDE SCENARIO 3: " << lin << "\n";
        }
    }

    // If goalRange < 0, the QRCode is too close -> MOVE BACKWARDS!
    else
    {
        //std::cout << "Goal Range < 0" << std::endl;
        myfile << "GOAL RANGE < 0 \n";

        // If the angle is relatively small, go straight back
        if(-REVERSE_ANGULAR_CHANGE <= bearing && bearing <= REVERSE_ANGULAR_CHANGE)
        {
            // Set Linear Velocity to be distance to goal point - targest distance m/s
            lin = goalRangeMultiplier_BACKWARDS;
            if(lin < -V_MAX)
            {
                lin = -V_MAX;
            }
            else if(lin > -V_MIN)
            {
                lin = -V_MIN;
            }
            ang = 0;
            myfile << "SCENARIO 1 \n";
        }

        // If bearing error is between 0 and PI, rotate clockwise (w < 0)
        // Let the bearing error be the input angular velocity, scale linear based on gamma.
        else if(REVERSE_ANGULAR_CHANGE < bearing && bearing <= M_PI)
        {
            myfile << "SCENARIO 2 \n";
            lin = goalRangeMultiplier_BACKWARDS;
            if(lin < -V_MAX)
            {
                lin = -V_MAX;
            }
            else if(lin > -V_MIN)
            {
                lin = -V_MIN;
            }

            ang = lin*gamma;
            if(ang < -W_MAX)
            {
                ang = -W_MAX;
            }
            myfile << "LIN INSIDE SCENARIO 2: " << lin << "\n";
        }

        // If bearing error is between 0 and -PI, rotate clockwise (w > 0)
        // Let the bearing error be the input angular velocity, scale linear based on gamma.
        else if(bearing < -REVERSE_ANGULAR_CHANGE && bearing >= -M_PI)
        {
            myfile << "SCENARIO 3 \n";
            lin = goalRangeMultiplier_BACKWARDS;
            if(lin < -V_MAX)
            {
                lin = -V_MAX;
            }
            else if(lin > -V_MIN)
            {
                lin = -V_MIN;
            }

            ang = -(lin*gamma);
            if(ang > W_MAX)
            {
                ang = W_MAX;
            }
            myfile << "LIN INSIDE SCENARIO 3: " << lin << "\n";
        }
    }

    myfile << "Linear Velocity = " << lin << "\n";
    myfile << "Angular Velocity = " << ang << "\n";
    myfile << "\n";
}

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
void FetchMovement::rpyFromPose(geometry_msgs::PoseStamped pose, double &fetchRoll, double &fetchPitch, double &fetchYaw)
{
    // Calc Roll, Pitch, Yaw of a given Pose
    // SOURCE: https://answers.ros.org/question/238083/how-to-get-yaw-from-quaternion-values/ (Marcoarruda - Aug. 10, 2017)
    double w = pose.pose.orientation.w;
    double x = pose.pose.orientation.x;
    double y = pose.pose.orientation.y;
    double z = pose.pose.orientation.z;
    // Create instance of Quaternion
    tf::Quaternion q(x, y, z, w);
    // Create 3x3 Rotation Matrix of Pose from Quaternion
    tf::Matrix3x3 m(q);
    // Extract Roll, Pitch and Yaw from 3x3 Rotation Matrix
    m.getRPY(fetchRoll, fetchPitch, fetchYaw);
}

/*! @brief Converts an angle outside of the range -2PI to 2PI radians to the equivalent angle
 *  inside that range.
 *
 *  @param[in] theta - The angle to be mapped inside the range -2PI to 2PI radians.
 *
 *  @return double - Function will return the input angle in radians mapped to a value in the
 *  range -2PI to 2PI.
 */
double FetchMovement::normaliseAngle(double theta)
{
    if(theta > (2 * M_PI))
    {
        theta = theta - (2 * M_PI);
    }
    else if(theta < 0)
    {
        theta = theta + (2 * M_PI);
    }
    return theta;
}
