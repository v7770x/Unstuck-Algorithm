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
    double x1 = msg.pose.pose.orientation.x;
    double y1 = msg.pose.pose.orientation.y;
    double z1 = msg.pose.pose.orientation.z;
    double w1 = msg.pose.pose.orientation.w;


    ROS_INFO("orientation: (%d, %d, %d, %d)", x1, y1, z1, w1);
    geometry_msgs::Twist nextTwist;
    nextTwist.linear.x= 0.1;
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

    pub  = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    sub = n.subscribe("/odom", 1, turtleCallback);
    ros::Rate loop_rate(1);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}