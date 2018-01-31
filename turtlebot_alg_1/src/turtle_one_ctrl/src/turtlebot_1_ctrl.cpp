#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

// #include <stdio.h>
// #include <time.h>
// #include <stdlib.h>

ros::Subscriber sub;
ros::Publisher pub;

void turtleCallback (const nav_msgs::Odometry&msg)
{
    int x = msg.pose.pose.position.x;
     int y = msg.pose.pose.position.y;
    ROS_INFO("food (%d, %d)", x, y);
    geometry_msgs::Twist nextTwist;
    nextTwist.linear.x= 2;
    nextTwist.linear.y= 2;
    nextTwist.linear.z= 2;

    // srand(time(NULL));
    // double ang = rand()%20 *1.0 /10;

    nextTwist.angular.z = 0.1;
    nextTwist.angular.x = 0;
    nextTwist.angular.y = 0;
    pub.publish(nextTwist);
}

int main(int a, char ** b)
{
    ros::init(a, b, "BobTheTurtle");

    ros::NodeHandle n;

    pub  = n.advertise<geometry_msgs::Twist>("/mobile_bse/commands/velocity",1);
    sub = n.subscribe("/odom", 1, turtleCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;

}