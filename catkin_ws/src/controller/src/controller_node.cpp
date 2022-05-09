#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <sstream>

#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

#include "fetchmovement.h"

#define QR_CODE_WIDTH 15.2          // Width of the QR Code in CM
#define DIST_SCALE_CONSTANT 0.2

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped fetchPose;
sensor_msgs::LaserScan base_scan;
double current_status = 0.0;
const double DIST_SCALE = QR_CODE_WIDTH*DIST_SCALE_CONSTANT;

bool collDetect = false;
bool inWallFollow = false;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose.pose.position.x = msg.get()->pose.position.x;
  pose.pose.position.y = msg.get()->pose.position.y;
  pose.pose.position.z = msg.get()->pose.position.z;
  pose.pose.orientation = msg.get()->pose.orientation;
}

void chatterCallback2(const std_msgs::Int8::ConstPtr& status)
{
    current_status = status.get()->data;
}

double quaternionToYaw(geometry_msgs::Quaternion q)
{
    return std::atan2(2.0f * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}

void odomCallback(const nav_msgs::Odometry& r)
{
    fetchPose.pose = r.pose.pose;
}

void laserCallback(const sensor_msgs::LaserScan& msg)
{
    //base_scan.ranges = msg.ranges;
    base_scan = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/visp_auto_tracker/object_position", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("/visp_auto_tracker/status", 1000, chatterCallback2);
  ros::Subscriber fetch_odom = n.subscribe("/odom", 1000, odomCallback);
  ros::Subscriber fetch_basescan = n.subscribe("/base_scan", 1000, laserCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel1", 1000);

  ros::Rate loop_rate(10);
  double something = abs(10);

  FetchMovement fetchM;
  bool inIRL = true;           // Boolean to determine whether the program is being run in the sim vs IRL
  double lastX = 0;             // Intialising value to track the most recent X Coordinate Value (relative to Fetch)
  double lastYaw = 0;           // Initialising value to track the Yaw of the robot if it loses track of the QR Code
  double previous_status = 0;    // Initialising value to keep track of the previous status of tracking the QR Code

  while (ros::ok())
  {
    // Variable to contol fetch linear + angular velocity
    double lin = 0;
    double ang = 0;

    // Calc Roll, Pitch, Yaw
    // Calcing Pitch to get Orientation of QR Code
    // SOURCE: https://answers.ros.org/question/238083/how-to-get-yaw-from-quaternion-values/ (Marcoarruda - Aug. 10, 2017)
    double roll, pitch, yaw;
    fetchM.rpyFromPose(pose, roll, pitch, yaw);

    // Calc Rotation of Fetch from Odom Pose
    double fetchRoll, fetchPitch, fetchYaw;
    fetchM.rpyFromPose(fetchPose, fetchRoll, fetchPitch, fetchYaw);

    // Variable to send velocity messages to Fetch controller
    geometry_msgs::Twist msg2;

    // Getting distance and angle variables for Pure Pursuit
    double distX = DIST_SCALE*pose.pose.position.x;
    double distZ = DIST_SCALE*pose.pose.position.z;
    double range = std::sqrt(std::pow(distX, 2) + std::pow(distZ, 2));
    double angle = atan2(pose.pose.position.x, pose.pose.position.z);

    // Print to Terminal

//    ROS_WARN("QR x pos: [%f]", pose.pose.position.x);
//    ROS_WARN("QR y pos: [%f]", pose.pose.position.y);
//    ROS_WARN("QR z pos: [%f]", pose.pose.position.z);
//    ROS_WARN("range: [%f]", range);
//    ROS_WARN("angle (in deg): [%f]", angle*180/M_PI);
//    ROS_WARN("pitch (in rad): [%f]", pitch);
//    ROS_WARN("pitch (in deg): [%f]", pitch*180/M_PI);
    ROS_WARN("Fetch x pos: [%f]", fetchPose.pose.position.x);
    ROS_WARN("Fetch y pos: [%f]", fetchPose.pose.position.y);
    ROS_WARN("Fetch z pos: [%f]", fetchPose.pose.position.z);
//    ROS_WARN("Fetch Yaw (in rad): [%f]", fetchYaw);
    ROS_WARN("Fetch Yaw (in deg): [%f]", fetchYaw*180/M_PI);
//    ROS_WARN("Current Status: [%f]", current_status);
//    ROS_WARN("Ranges: [%f]", base_scan.ranges.at(0));


    // If robot can detect QR code
    if((current_status == 0 || current_status == 1) && collDetect == false)
    {
        //ROS_WARN("NO SIGHT");
        lin = 0;
        ang = 0;
    } else {
        //fetchM.purePursuit2(range, distX, angle, lin, ang, current_status);
        collDetect = true;
    }
    // Calculating linear + angular velocity with Pure Pursuit


    // Initialise variables for closest range and its angle. Callback data from laserscan base to find closest distance and sampled angle.
    double closest_range = 5;
    double smallest_angle = 0;

    for (unsigned int i=0; i < base_scan.ranges.size(); i++) {  //size is 662

        if (base_scan.ranges.at(i) < closest_range){
            closest_range = base_scan.ranges.at(i);
            smallest_angle = i*base_scan.angle_increment*180/M_PI;
        }
    }

    /*
    // If within 0.5m of an obstacle, trigger collision detection mode
    if(closest_range < 0.5)
    {
        collDetect = true;
    }
    else
    {
        collDetect = false;
    }
    */


    // If collision detection mode is triggered:
    if (collDetect == true)
    {

        //if (current_status == 3) fetchM.purePursuit2(range, distX, angle, lin, ang, current_status);

        if (current_status == 3)
        {
            // If previously in wallFollow mode, turn mode off.
            if(inWallFollow == true)
            {
                inWallFollow = false;
            }

            // DODGE and PURSUE
            fetchM.purePursuit2(range, distX, angle, lin, ang, current_status);

            //ROS_INFO_STREAM("Smallest angle: " << smallest_angle << "and closest range is " << closest_range);
            if (smallest_angle > 0 && smallest_angle <= 110) {
                //ROS_INFO_STREAM("OBJECT ON LEFT");
                //ROS_INFO_STREAM("LEFT SIDE: " << closest_range);
                if (closest_range < 0.5){ang = -0.2; lin = 0.2;}
                //else if (base_scan.ranges.at(20/0.33) >= 0.55 && base_scan.ranges.at(20/0.33) <= 0.8) {ang = 0; lin = 0.3;}
                //else if (base_scan.ranges.at(20/0.33) > 0.8  && base_scan.ranges.at(20/0.33) < 1){ang = 0.2; lin = 0.3;}


            }
            else if (smallest_angle > 110 && smallest_angle < 220) {
                //ROS_INFO_STREAM("OBJECT ON RIGHT");
                //ROS_INFO_STREAM("RIGHT SIDE: " << closest_range);
                if (closest_range < 0.5){ang = 0.2; lin = 0.2;}
                //else if (base_scan.ranges.at(110/0.33) >= 0.55 && base_scan.ranges.at(110/0.33 <= 0.8)) {ang = 0; lin = 0.3;}
                //else if (base_scan.ranges.at(110/0.33) > 0.8 && base_scan.ranges.at(110/0.33 < 1)){ang = -0.2; lin = 0.3;}

            }
        }


        // Different Codes for IRL vs Sim
        if(inIRL == true)
        {
            // If we have just lost sight of the QR code, get the current Yaw of the Robot
            if(previous_status == 3 && current_status != 3)
            {
                lastYaw = fetchYaw;
            }

            // If the robot can't see the QR Code, start spinning to try and regain view of the QR code (lin is initialised = 0)  
            if(current_status != 3)
            {		
                if(lastX < 0)
                {   ROS_INFO_STREAM("Rotating CCW. " << "Angle is " << abs(lastYaw-fetchYaw)*180/M_PI);
                    // If Fetch has rotated approx. > 90 deg, stop rotating (prevent eternal rotation)
                    if(abs(lastYaw - fetchYaw)*180/M_PI > 90)
                    {
                        ang = 0;
                    }
                    else
                    {	
                        // Rotate CCW
                        ang = 0.2;
                    }
                }
                else if(lastX > 0)
                {   ROS_INFO_STREAM("Rotating CW. " << "Angle is " << abs(lastYaw-fetchYaw)*180/M_PI);
                    // If Fetch has rotated approx. > 90 deg, stop rotating (prevent eternal rotation)
                    if(abs(lastYaw - fetchYaw)*180/M_PI > 90)
                    {

                        ang = 0;
                    }
                    else
                    {
                        // Rotate CW
                        ang = -0.2;
                    }
                }
            }
        }


       // if
       // {
            if((current_status != 3 && ((lastX < 0 && base_scan.ranges.at(20/0.33) < 1) || (lastX > 0 && base_scan.ranges.at(200/0.33) < 1))) && inWallFollow == false)
            // If the robot loses sight of the QR Code, go into wall follow mode.
            //else if ((current_status != 3 && closest_range < 1) && inWallFollow == false)
            {
                // Loses sight
                inWallFollow = true;
            }
            if(inWallFollow == true)
            {
                // WALL FOLLOW
                // lin = 0; ang = 0;
                //ROS_WARN("CLOSEST RANGE: [%f]: ", closest_range);
                //ROS_INFO_STREAM("RIGHT SIDE: " << base_scan.ranges.at(200/0.33));
                //ROS_INFO_STREAM("DEAD AHEAD: " << base_scan.ranges.at(110/0.33));
                //ROS_INFO_STREAM("ANGLE: " << smallest_angle);

                if (smallest_angle > 0 && smallest_angle <= 110)
                {
                    //ROS_INFO_STREAM("OBJECT ON LEFT. WALL FOLLOWING");
                    if (closest_range < 0.5){ang = -0.2; lin = 0.3;}
                    else if (closest_range >= 0.5 && closest_range <= 0.8) {ang = 0; lin = 0.3;}
                    else if (closest_range > 0.8  && closest_range < 1){ang = 0.2; lin = 0.3;}

                    if (base_scan.ranges.at(20/0.33) >= 1){ang = 0.5; lin = 0.3;}
                    if (base_scan.ranges.at(110/0.33) <= 0.8) {ang = -0.5;}

                }
                else if (smallest_angle > 110 && smallest_angle < 220)
                {
                    //ROS_INFO_STREAM("OBJECT ON RIGHT. WALL FOLLOWING");
                    if (closest_range < 0.5){ang = 0.2; lin = 0.3;}
                    else if (closest_range >= 0.5 && closest_range <= 0.8) {ang = 0; lin = 0.3;}
                    else if (closest_range > 0.8 && closest_range < 1){ang = -0.2; lin = 0.3;}


                    if (base_scan.ranges.at(200/0.33) >= 1){ang = -0.5; lin = 0.3;}
                    if (base_scan.ranges.at(110/0.33) <= 0.8) {ang = 0.5;}
                }

            }
      //  }
        
    }



    //ROS_INFO_STREAM("Smallest angle: " << smallest_angle << "and closest range is " << closest_range);

    // Update lastX (if current_status == 3)
    if(current_status == 3)
    {
        lastX = distX;
    }
    // Update previous_status
    previous_status = current_status;
    

    // Publish calculated linear/angular velocities
    msg2.linear.x = lin;
    msg2.angular.z = ang;

//    ROS_WARN("LINEAR VELOCITY: [%f]", lin);
//    ROS_WARN("ANGULAR VELOCITY: [%f]", ang);

    pub.publish(msg2);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}

