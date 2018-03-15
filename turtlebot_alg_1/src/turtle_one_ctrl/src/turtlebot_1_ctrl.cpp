#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <std_msgs/String.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include<deque>

using namespace std;

// #include <stdio.h>
// #include <time.h>
// #include <stdlib.h>

ros::Subscriber sub;
ros::Publisher pub, statePub;
nav_msgs::Odometry prevOdom;
nav_msgs::Odometry originalStuckOdom;


// AlgPart1
double roll, pitch, yaw;
double backDistance;
bool turned(double targetAngle, const nav_msgs::Odometry& currPos);


double initialYaw;
int cnt = 0;
int state, subState;

//algPart2
deque <nav_msgs::Odometry>lastTwentySec;
bool rightCheck;
bool distance(const nav_msgs::Odometry & currOdom);

void turtleCallback(const nav_msgs::Odometry&msg);
void extractRPY(double & r, double & p, double& y, const nav_msgs::Odometry& msg); // converts the odometry message to roll pitch and yaw
// void extractPosition()
void sampleTurn (double angle, double yaw, double initialYaw);

//base functions
bool checkIfStuck();
void move(double linSpeed, double angularSpeed);
bool atDestination(double distance, const nav_msgs::Odometry& currPos, double yaw);//maybe instead take in the aldready processed x and y coordinates
bool turned(double targetAngle, double initYaw, double yaw); // maybe instead take in yaw; see sample turn for example of how to do it
void updatePrevOdom(const nav_msgs::Odometry& msg);
void updateStateMsg(string message); // to publish a message to the statePub channel

//algorithm functions & variables
void algorithmPart1(const nav_msgs::Odometry & msg, double yaw);
void algorithmPart2(const nav_msgs::Odometry& msg);
void algorithmPart3();
clock_t alg3Timer;
int alg3Count;
float angDenom, linDenom;



int main(int a, char ** b)
{
    ros::init(a, b, "BobTheTurtle");

    ros::NodeHandle n;
    //cnt = 0;

    pub  = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    statePub = n.advertise<std_msgs::String>("/unstuck_node/message",1);
bool sub = n.subscribe("/odom", 1, turtleCallback);
    state = subState = 0;
    state =3;


    //alg3 initialization
    alg3Count = 0;
    alg3Timer=0;  
    angDenom = 500;
    linDenom = 5000;

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

    // int x = msg.pose.pose.position.x;
    // int y = msg.pose.pose.position.y;
    // double roll, pitch, yaw;
    // extractRPY(roll,pitch,yaw, msg);

    if (cnt == 0)
    {
        updatePrevOdom(msg);
        cnt = 1;
    }

    if(lastTwentySec.size() == 20)
		lastTwentySec.pop_front();
	lastTwentySec.push_back(msg);

    //
    // if(cnt<200)
    // move(-0.5,0);
    // else
    // move(0.5,0);

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
        switch(state)
        {
            case 1:
                algorithmPart1(msg,yaw);
                break;
            case 2:
                algorithmPart2(msg);
                break;
            case 3:
                algorithmPart3();
                break;

        }
        
    }


    // sampleTurn(M_PI/2, yaw, initialYaw);

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

bool atDestination(double distance, const nav_msgs::Odometry& currPos, double yaw)
{
    double xCurrent = currPos.pose.pose.position.x;
    double yCurrent = currPos.pose.pose.position.y;
    double xPrev = prevOdom.pose.pose.position.x;
    double yPrev = prevOdom.pose.pose.position.y;
    double xDest = xPrev + distance*cos(yaw);
    double yDest = yPrev + distance*sin(yaw);
    ROS_INFO("xCurr: %.2f", xCurrent);
    ROS_INFO("x: %.2f",xDest - xCurrent);
    ROS_INFO("y: %.2f",yDest - yCurrent);


    if (fabs(xDest - xCurrent) < 1e-1 && fabs(yDest - yCurrent) < 1e-1)
    {
    	updatePrevOdom(currPos);
        return true;
    }
    
    return false;
}

bool turned(double targetAngle, const nav_msgs::Odometry& currPos)
{
	double targetYaw = initialYaw + targetAngle; //Angle from positive x-axis after turning the desired "targetAngle"
    double r,p,y;
    extractRPY(r,p,y,currPos);
	if (fabs(targetYaw - y) < 1e-2)
		return true;

	return false;
}

bool turned(double targetAngle, double initYaw, double yaw)
{
	double targetYaw = initialYaw + targetAngle; //Angle from positive x-axis after turning the desired "targetAngle"

	if (fabs(targetYaw - yaw) < 1e-2)
		return true;

	return false;
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
        // if(lastTwentySec.size() == 20)
        // {
        // 	double x = lastTwentySec.front().pose.pose.position.x;
        // 	double y = lastTwentySec.front().pose.pose.position.y;
        // 	double x2 = lastTwentySec.back().pose.pose.position.x;
        // 	double y2 = lastTwentySec.back().pose.pose.position.y;
        // 	return hypot((x2-x), (y2-y)) < 2; // placeholder number
        // }
	return false;
}

bool distance(const nav_msgs::Odometry & currOdom)
{
	double x = currOdom.pose.pose.position.x;
	double y = currOdom.pose.pose.position.y;
	double x2 = prevOdom.pose.pose.position.x;
	double y2 = prevOdom.pose.pose.position.y;
	return hypot((x2-x), (y2-y));
}


void algorithmPart1 (const nav_msgs::Odometry & msg, double yaw)
{
	switch (subState)
	{
		//Move back 5 m
		case 0:
			if (!atDestination(2, msg, yaw))
			{
				ROS_INFO("Moving");
				move(0.2, 0);
			}
			else
			{
				ROS_INFO("Done moving");
				subState = 1;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			//If can't move back 5 m successfully, change state to 2 
			if (checkIfStuck())//add time
			{
				state = 2;
				subState = 0;
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
				ROS_INFO("Stuck");
			}

			break;
		//Turn right 45, assume successful
		case 1:
			if (!turned(-M_PI/4, initialYaw, yaw))// && !turnedYet) 
			{
				ROS_INFO("TURNING");
				move(0, -0.3);
			}
			else
			{
				ROS_INFO("DONE TURNING");
				subState = 2;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			break;
		//Go forward 
		case 2:
			if (!atDestination(5/cos(M_PI/4), msg, yaw))
			{
				ROS_INFO("Moving");
				move(0.2, 0);
			}
			else //*****UNSTUCK*****
			{
				ROS_INFO("Done moving");
				state=0;
                subState=0;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			if (checkIfStuck())//time delay
			{
				subState = 3;
				updatePrevOdom(msg);
				ROS_INFO("Stuck");
				initialYaw - yaw;
				extractRPY(roll, pitch, yaw, msg);
				move(0, 0);
				backDistance = fabs(msg.pose.pose.orientation.x - prevOdom.pose.pose.orientation.x) / cos(yaw);
			}

			break;
		//Stuck from case 2, go back to prevOdom
		// case 3:
        //     state = 0;
		// 	if (!atDestination(-backDistance, msg, yaw))
		// 	{
		// 		ROS_INFO("Moving");
		// 		move(-0.2, 0);
		// 	}
		// 	else
		// 	{
		// 		ROS_INFO("Done moving");
		// 		subState = 4;
		// 		move(0, 0);
		// 		updatePrevOdom(msg);
		// 		initialYaw = yaw;
		// 		extractRPY(roll, pitch, yaw, msg);
		// 	}

		// 	break;
		// case 4:
		// 	if (!turned(M_PI/2, initialYaw, yaw))
		// 	{
		// 		ROS_INFO("Moving");
		// 		move(0, 0.3);
		// 	}
		// 	else
		// 	{
		// 		ROS_INFO("Done moving");
		// 		subState = 5;
		// 		move(0, 0);
		// 		updatePrevOdom(msg);
		// 		initialYaw = yaw;
		// 		extractRPY(roll, pitch, yaw, msg);
		// 	}

		// 	break;
		// case 5:
		// 	if (!atDestination(5/cos(M_PI/4), msg, yaw))
		// 	{
		// 		ROS_INFO("Moving");
		// 		move(0.2, 0);
		// 	}
		// 	else //*****UNSTUCK*****
		// 	{
		// 		ROS_INFO("Done moving");
		// 		//subState = 4;
		// 		move(0, 0);
		// 		updatePrevOdom(msg);
		// 		initialYaw = yaw;
		// 		extractRPY(roll, pitch, yaw, msg);
		// 	}

		// 	if (checkIfStuck())
		// 	{
		// 		subState = 6;
		// 		updatePrevOdom(msg);
		// 		ROS_INFO("Stuck");
		// 		initialYaw - yaw;
		// 		extractRPY(roll, pitch, yaw, msg);
		// 		move(0, 0);
		// 		backDistance = fabs(msg.pose.pose.orientation.x - prevOdom.pose.pose.orientation.x) / cos(yaw);
		// 	}

		// 	break;
		// case 6:
		// 	if (!atDestination(-backDistance, msg, yaw))
		// 	{
		// 		ROS_INFO("Moving");
		// 		move(-0.2, 0);
		// 	}
		// 	else
		// 	{
		// 		ROS_INFO("Done moving");
		// 		subState = 4;
		// 		move(0, 0);
		// 		updatePrevOdom(msg);
		// 		initialYaw = yaw;
		// 		extractRPY(roll, pitch, yaw, msg);
		// 	}

		// 	break;
	}
}
void algorithmPart2(const nav_msgs::Odometry& msg)
{
	if(subState == 0) //TODO:FINISH 
	{

		move(0, 60);
		if (turned(90, msg))
		{
			subState = 1;
			updatePrevOdom(msg);
		}
	}
	if(subState == 1)
	{
		move(60, 0);
		if(atDestination(5, msg, yaw))
		{
			subState = 2;
			updatePrevOdom(msg);
		}
		if(checkIfStuck())
		{
			if(rightCheck)
			{
				updatePrevOdom(msg);
				state = 3;
				subState = 0;
			}
			else
			{
				rightCheck = true;
				updatePrevOdom(msg);
				subState = 6;
			}
		}
	}
	if(subState == 2)
	{
		int multi = 1;
		if(rightCheck)
			multi = -1;
		move(0, multi); // turns left or right based on whch direction is being checked
		if (turned(-90*multi, msg))
		{
			subState = 3;
			updatePrevOdom(msg);
		}
	}
	if(subState == 3)
	{
		move(60, 0);
		if(atDestination(10, msg,yaw))
		{
			state = subState = 0;
			updatePrevOdom(msg);
		}
		if(checkIfStuck())
		{
			subState = 4;
		}
	}
	if(subState == 4)
	{
		move(-60, 0);
		if(distance(msg) < 0.5)
		{
			subState = 5;
			updatePrevOdom(msg);
		}
	}
	if(subState == 5)
	{
		int multi = 1;
		if(rightCheck)
			multi = -1;
		move(0, -60*multi); // turns left or right based on whch direction is being checked
		if (turned(-90*multi, msg))
		{
			subState = 1;
			updatePrevOdom(msg);
		}
	}
	
	
	if(subState == 6)
	{
		move(0, 60);
		if (turned(180, msg))
		{
			subState = 1;
			updatePrevOdom(msg);
		}
	}
}

void algorithmPart3()
{
    if(checkIfStuck()) //make into !checkIfStuck()
    {
        state = 0;
        subState = 0;
        angDenom = 500;
        linDenom = 5000;
        return;
    }
    switch(subState)
    {
        case 0:
        { 
            srand(time(NULL));
            double randAng= (rand()%1000)/angDenom+.5;
            
            if(alg3Count==0)
            {
                // alg3Count=1;
                ROS_INFO("Initial: %d",clock());
                alg3Timer= clock();
            }
            clock_t deltaTime= (clock()-alg3Timer);
            float deltaSecs = (float)deltaTime*100/CLOCKS_PER_SEC;
            if((int)deltaSecs%4>2)
                alg3Count=2;
            else
                alg3Count =1;
            
            if( deltaSecs>30)
            {
                alg3Count =0;
                alg3Timer=0;
                subState =1;
            }
            else if(alg3Count == 1)
            {
                ROS_INFO("right");
                move(0,randAng);
                // ROS_INFO("%.3f",deltaSecs);
            } else if(alg3Count ==2)
            {
                ROS_INFO("left");
                move(0,-randAng);
            }
            
        }
        break;
        case 1:
        {
            srand(time(NULL));
            double randLin= (rand()%1000)/linDenom+0.05;
            if(alg3Count==0)
                alg3Timer= clock();
            clock_t deltaTime= (clock()-alg3Timer);
            float deltaSecs = (float)deltaTime*100/CLOCKS_PER_SEC;
            if((int)deltaSecs%4>2)
                alg3Count=2;
            else
                alg3Count =1;
            if( deltaSecs>10)
            {
                alg3Count =0;
                alg3Timer=0;
                subState =2;
            }
            else if(alg3Count == 1)
            {
                ROS_INFO("forward");
                move(randLin,0);
            } else if(alg3Count ==2)
            {
                ROS_INFO("backward");
                move(-randLin,0);
            }
        
        }
        break;
        case 2:
        {
            srand(time(NULL));
            double randLin= (rand()%1000)/linDenom-0.1;

            srand(time(NULL));
            double randAng= (rand()%1000)/angDenom-1;
            if(alg3Count==0)
            {
                alg3Timer= clock();
                alg3Count++;
            }
                
            clock_t deltaTime= (clock()-alg3Timer);
            float deltaSecs = (float)deltaTime*100/CLOCKS_PER_SEC;
            if( deltaSecs>30)
            {
                alg3Count =0;
                alg3Timer=0;
                subState =3;
            }
            ROS_INFO("RandBoth");
            move(randLin,randAng);
            
        }
        break;
        case 3:
        {
            if(alg3Count==0)
            {
                alg3Timer= clock();
                alg3Count++;
            }
            clock_t deltaTime= (clock()-alg3Timer);
            float deltaSecs = (float)deltaTime*100/CLOCKS_PER_SEC;
            if( deltaSecs>20)
            {
                alg3Count =0;
                alg3Timer=0;
                subState =4;
            }else if(deltaSecs>50)
            {
                ROS_INFO("backward");
                move(-1000/linDenom, 0);  
            }
            else
            {
                ROS_INFO("forward");
                move(1000/linDenom,0);    
            }

        }
        break;
        case 4:
        {
            linDenom/=1.4;
            angDenom/=1.4;
            subState=0;
        }
        break;
        default:
        {
            state=0;
            ROS_INFO("OUT");
        }
        break;

        
    }
    
    return;
}