#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <cmath>
using namespace std;

// #include <stdio.h>
// #include <time.h>
// #include <stdlib.h>

ros::Subscriber sub;
ros::Publisher pub;
geometry_msgs::Twist prevTwist, originalStuckTwist;

// deque lastTwentySec;
double initialYaw;
int cnt = 0;
int state, subState;

void turtleCallback(const nav_msgs::Odometry&msg);
void extractRPY(double & r, double & p, double& y, const nav_msgs::Odometry& msg);
geometry_msgs::Twist sampleTurn (double angle, double yaw, double initialYaw);

//base functions
bool checkIfStuck();
void move(double linSpeed, double angularSpeed);
bool atDestination(double destination, const nav_msgs::Odometry& currPos); //maybe instead take in the aldready processed x and y coordinates
bool turned(double targetAngle, const nav_msgs::Odometry& msg); // maybe instead take in yaw; see sample turn for example of how to do it
void updatePrevTwist(const nav_msgs::Odometry& msg);



int main(int a, char ** b)
{
    ros::init(a, b, "BobTheTurtle");

    ros::NodeHandle n;
    cnt = 0;

    pub  = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    sub = n.subscribe("/odom", 1, turtleCallback);
    ros::Rate loop_rate(60);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
  

void turtleCallback (const nav_msgs::Odometry&msg)
{
    //get all relevant odometry info:
    int x = msg.pose.pose.position.x;
    int y = msg.pose.pose.position.y;
    double roll, pitch, yaw;
    extractRPY(roll,pitch,yaw, msg);

    if(cnt == 0)
        initialYaw = yaw;
    cnt++;

    //

    // if(checkIfStuck() && state == 0)
    // {
    //     state = 1;
    //     substate = 0;
    // }else
    // {
    //     // publish(unstuck)
    // }
    // if(state != 0)
    // {
    //     // publish(stuck)
        
    // }


    geometry_msgs::Twist nextTwist = sampleTurn(M_PI/2, yaw, initialYaw);

    pub.publish(nextTwist);
}

void extractRPY(double & r, double & p, double& y, const nav_msgs::Odometry& msg)
{
    double x1 = msg.pose.pose.orientation.x;
    double y1 = msg.pose.pose.orientation.y;
    double z1 = msg.pose.pose.orientation.z;
    double w1 = msg.pose.pose.orientation.w;
    tf::Quaternion q(x1,y1,z1,w1);
    tf::Matrix3x3 mat(q);
    mat.getRPY(r,p,y);

}

geometry_msgs::Twist sampleTurn (double angle, double yaw, double initialYaw)
{
    geometry_msgs::Twist nextTwist;
    nextTwist.linear.x= 0;
    nextTwist.linear.y= 0;
    nextTwist.linear.z= 0;

    // srand(time(NULL));
    // double ang = rand()%20 *1.0 /10;

    nextTwist.angular.z = 0.2;
    nextTwist.angular.x = 0;
    nextTwist.angular.y = 0;

    if(fabs( fabs(yaw-initialYaw) - angle) < 1e-1)
    {
        
        nextTwist.angular.z = 0;
        nextTwist.linear.x= 0;
        ROS_INFO("%.2d",fabs(yaw-initialYaw));
        ROS_INFO("Reached!");

    }
    return nextTwist;
}
