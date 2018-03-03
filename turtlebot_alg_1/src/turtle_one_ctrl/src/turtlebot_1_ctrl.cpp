#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <std_msgs/String.h>
using namespace std;

// #include <stdio.h>
// #include <time.h>
// #include <stdlib.h>

ros::Subscriber sub;
ros::Publisher pub, statePub;
nav_msgs::Odometry prevOdom;
nav_msgs::Odometry originalStuckOdom;

// deque lastTwentySec;
double initialYaw;
int cnt = 0;
int state, subState;

void turtleCallback(const nav_msgs::Odometry&msg);
void extractRPY(double & r, double & p, double& y, const nav_msgs::Odometry& msg); // converts the odometry message to roll pitch and yaw
// void extractPosition()
void sampleTurn (double angle, double yaw, double initialYaw);

//base functions
bool checkIfStuck();
void move(double linSpeed, double angularSpeed);
bool atDestination(double destination, const nav_msgs::Odometry& currPos); //maybe instead take in the aldready processed x and y coordinates
bool turned(double targetAngle, const nav_msgs::Odometry& msg); // maybe instead take in yaw; see sample turn for example of how to do it
void updatePrevOdom(const nav_msgs::Odometry& msg);
void updateStateMsg(string message); // to publish a message to the statePub channel




int main(int a, char ** b)
{
    ros::init(a, b, "BobTheTurtle");

    ros::NodeHandle n;
    cnt = 0;

    pub  = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    statePub = n.advertise<std_msgs::String>("/unstuck_node/message",1);

    sub = n.subscribe("/odom", 1, turtleCallback);
    state = subState = 0;

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

    if(checkIfStuck() && state == 0)
    {
        state = 1;
        subState = 0;
    }else
    {
        updateStateMsg("unstuck");
    }
    if(state != 0)
    {
        updateStateMsg("stuck");
        
    }


    sampleTurn(M_PI/2, yaw, initialYaw);

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

void extractPosition(double & x, double & y , double & z, const nav_msgs::Odometry&msg)
{


}

void sampleTurn (double angle, double yaw, double initialYaw)
{
    double  ang = 0.2;
    if(fabs( fabs(yaw-initialYaw) - angle) < 1e-1)
    {
        
        ang = 0;
        ROS_INFO("Reached!");
    }
    move(0,ang);
    // return nextTwist;
}

void move(double linSpeed, double angularSpeed)
{
    geometry_msgs::Twist nextTwist;
    nextTwist.linear.x= linSpeed;
    nextTwist.linear.y= 0;
    nextTwist.linear.z= 0;

    nextTwist.angular.z = angularSpeed;
    nextTwist.angular.x = 0;
    nextTwist.angular.y = 0;

    pub.publish(nextTwist);
}

void updatePrevOdom(const nav_msgs::Odometry & currOdom)
{
    prevOdom = currOdom;
}

void updateStateMsg(string msg)
{
    
    std_msgs::String s;
    s.data = msg;
    statePub.publish(s);
    // ROS_INFO("%s",s.data.c_str());

}

bool checkIfStuck()
{
    return 0;
}

