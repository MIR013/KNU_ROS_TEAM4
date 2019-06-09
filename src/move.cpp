#include <stdio.h>

 

#include <stdlib.h>

 

#include <sys/types.h>

 

#include <sys/stat.h>

 

#include <fcntl.h>

 

#include <ros/ros.h>

 

#include <geometry_msgs/Twist.h>

 

#include <nav_msgs/Odometry.h>

 

#include <boost/thread/mutex.hpp>

 

#include <tf/tf.h>

 

#include <math.h>

 

#include <cmath>

#include <unistd.h>

 

#define MAX 1024

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

//

 

#define toRadian(degree)	((degree) * (M_PI / 180.))

 

#define toDegree(radian)	((radian) * (180. / M_PI))

 

 

 

 

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

// Global variable

 

boost::mutex mutex;

 

nav_msgs::Odometry g_odom;

 

float pre_dAngleTurned;

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

// callback function

 

void

 

odomMsgCallback(const nav_msgs::Odometry &msg)

 

{

 

    // receive a '/odom' message with the mutex

 

    mutex.lock(); {

 

        g_odom = msg;

 

    } mutex.unlock();

 

}

 

 

 

 

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

// Return current Transform Matrix information from the odom

 

tf::Transform

 

getCurrentTransformation(void)

 

{

 

    // transformation buffer

 

    tf::Transform transformation;

 

 

 

    // odom buffer

 

    nav_msgs::Odometry odom;

 

 

 

    // copy a global '/odom' message with the mutex

 

    mutex.lock(); {

 

        odom = g_odom;

 

    } mutex.unlock();

 

 

 

    // save position

 

    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

 

 

    //printf("Turtlebot x = %lf, y = %lf\n", odom.pose.pose.position.x,odom.pose.pose.position.y);

 

    // save Quaternion

 

    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

 

 

    //printf("getCurrent x = %lf, y = %lf, z = %lf\n", transformation.getOrigin().getX(), transformation.getOrigin().getY(), transformation.getOrigin().getZ());

 

    // Return

 

    return transformation;

 

}

 

 

 

 

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

// save position of stopped robot

 

tf::Transform

 

getInitialTransformation(void)

 

{

 

    // tf transform matrix

 

    tf::Transform transformation;

 

 

 

	// receive odomtry message about initial position

 

    ros::Rate loopRate(1000.0);

 

 

 

 

 

    while(ros::ok()) {

 

		// First, receive callback message

 

        ros::spinOnce();

 

 

 

        // get current transformationreturn;

 

        transformation = getCurrentTransformation();

 

 

 

		// if you got a message, break!

 

        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {

 

            break;

 

        } else {

 

            loopRate.sleep();

 

        }

 

    }

 

 

 

    // return

 

    return transformation;

 

}

 

 

 

 

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

// execute doRotation

 

bool

 

doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)

 

{

 

    //the command will be to turn at 'rotationSpeed' rad/s

 

    geometry_msgs::Twist baseCmd;

 

    baseCmd.linear.x = 0.0;

 

    baseCmd.linear.y = 0.0;

	printf("1. dRotation = %lf\n", dRotation);

 

    if(dRotation < 0.) {

 

        baseCmd.angular.z = -dRotationSpeed;

 

    } else {

 

       	baseCmd.angular.z = dRotationSpeed;

 

    }

 

 

 

	if(fabs(dRotation) >= M_PI) {

		if(dRotation > 0)

			dRotation -= M_PI;

		else

			dRotation += M_PI;

	}

	printf("2. dRotation = %lf\n", dRotation);

 

 

	// receive odometry message about initial position while moving

 

    bool bDone = false;

 

    ros::Rate loopRate(1000.0);

 

    double dAngleTurned;

 

 

 

    while(ros::ok() && !bDone) {

 

		// First, receive callback message

 

        ros::spinOnce();

 

 

 

        // get current transformation

 

        tf::Transform currentTransformation = getCurrentTransformation();

 

 

 

        //see how far we've traveled

    	tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation;

 

        tf::Quaternion rotationQuat = relativeTransformation.getRotation();

 

        

	// current theta -> atan2(sin, cos)

 

        dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));

 

	//printf("dRotation=%lf, dAngleTurned = %lf\n", dRotation, dAngleTurned);

 

 

  	// check termination condition

	    

/*

	if(theta > 0 && theta > pre_theta) {

 

		if(fabs(dAngleTurned) < fabs(dRotation)) {

			bDone = true;

			break;

		} else {

			pre_dAngleTurned = dAngleTurned;

 

            		//send the drive command

 

            		pubTeleop.publish(baseCmd);

			printf("here\n");

 

            		// sleep!

 

            		loopRate.sleep();

		}

	}

*/

 

 

    	if(fabs(dRotation - dAngleTurned) < 0.01) 

 

	{

 

            bDone = true;

 

            break;

 

        } else {

 

	    pre_dAngleTurned = dAngleTurned;

 

            //send the drive command

 

            pubTeleop.publish(baseCmd);

 

 

 

            // sleep!

 

            loopRate.sleep();

 

        }

 

	

 

    }

 

 

    // initialization

 

    baseCmd.linear.x = 0.0;

    baseCmd.linear.y = 0.0;

 

    baseCmd.angular.z = 0.0;

    //printf("Rotation: stop rotation!!\n");

 

    pubTeleop.publish(baseCmd);

 

 

 

    return bDone;

 

}

 

 

 

 

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

// Move

 

bool

 

doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double d_x, double d_y, double dTranslation, double dTranslationSpeed)

 

{

 

    //the command will be to go forward at 'translationSpeed' m/s

 

    geometry_msgs::Twist baseCmd;

 

 

 

    if(dTranslation < 0) {

 

        baseCmd.linear.x = -dTranslationSpeed;

 

    } else {

 

        baseCmd.linear.x = dTranslationSpeed;

 

    }

 

 

 

    baseCmd.linear.y = 0;

 

    baseCmd.angular.z = 0;

 

 

 

	// receive odometry message about current position while moving

 

    bool bDone = false;

 

    ros::Rate loopRate(1000.0);

 

 

    //tf::Transform relativeTransformation;

    //relativeTransformation.setOrigin(tf::Vector3((double)d_x, (double)d_y, 0));

 

 

    while(ros::ok() && !bDone) {

 

		// First, receive callback message

 

        ros::spinOnce();

 

 

 

        // get current transformation

 

        tf::Transform currentTransformation = getCurrentTransformation();

 

 

 

        //see how far we've traveled

 

        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;

 

 

 

        double dDistMoved = relativeTransformation.getOrigin().length();

	double cur_x = currentTransformation.getOrigin().getX();

	double cur_y = currentTransformation.getOrigin().getY();

	//printf("d_x=%lf, d_y=%lf\n, dist=%lf, dtranslation=%lf\n", d_x, d_y, dDistMoved, dTranslation);

 

 

        //double dDistMoved = sqrt(pow((cur_x - d_x), 2) + pow((cur_y - d_y), 2));

 

 

        // check termination condition

 

 

 

        if(fabs(dDistMoved) >= fabs(dTranslation)) {

 

            bDone = true;

   	    baseCmd.linear.x = 0.0;

    	    baseCmd.linear.y = 0.0;

 

    	    baseCmd.angular.z = 0.0;

 

    	    pubTeleop.publish(baseCmd);

 

 

            break;

 

        } else {

 

            //send the drive command

 

            pubTeleop.publish(baseCmd);

 

 

 

            // sleep!

 

            loopRate.sleep();

 

        }

 

    }

 

 

 

    //initialization

 

 

 

    baseCmd.linear.x = 0.0;

    baseCmd.linear.y = 0.0;

 

    baseCmd.angular.z = 0.0;

 

    pubTeleop.publish(baseCmd);

 

    printf("Translation: stop moving!!\n");

 

    return bDone;

 

}

 

 

 

 

 

 

 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

//

 

int main(int argc, char **argv)

 

{

 

    int i, fd, readn = 0;

 

    int cur_x, cur_y, read_x, read_y;

 

    double list_x[15] = {0, }, list_y[15] = {0, };

 

    int buf_size = 6;

 

    char buf[MAX];

 

    fd = open("/home/woo/catkin_ws/src/knu_ros_lecture/path.txt", O_RDONLY);

 

    if(fd == -1)

 

	printf("FD error...\n");

 

    else

 

	printf("FD Success..\n");

 

 

 

 

 

    

 

    for(i = 0; i < 12; i++) {

 

   	memset(buf, 0x00, MAX);

 

	strcpy(buf, "");

 

    	readn = read(fd, buf, 6);

 

	// x is positive number

 

	if(buf[1] != 45 && buf[2] != 45) {

 

    		read_x = buf[1] - '0';

 

 

 

		// y is positive number

 

		if(buf[3] != 45) {

 

    			read_y = buf[3] - '0';

 

		}

 

		// y is negative number

 

		else {

 

			read_y = (buf[4] - '0') * (-1);

 

			read(fd, buf, 1);

 

		}

 

	}

 

 

 

	// x is negative number

 

	else {

 

		read_x = (buf[2] - '0') * (-1);

 

 

 

		// y is positive number

 

		if(buf[4] != 45) {

 

    			read_y = buf[4] - '0';

 

			read(fd, buf, 1);

 

		}

 

		// y is negative number

 

		else {

 

			read_y = (buf[5] - '0') * (-1);

 

			read(fd, buf, 2);

 

		}

 

	}

 

	list_x[i] = read_x;

 

	list_y[i] = read_y;

 

    }

 

	

 

	

 

    // ROS initialization

 

    ros::init(argc, argv, "turtle_position_move");

 

 

 

    // Ros initialization

 

    ros::NodeHandle nhp, nhs;

 

 

 

    // Decleation of subscriber

 

    ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);

 

 

 

    // Create a publisher object

 

    ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

 

 

 

    double pre_theta = 0;

 

 

 

    for(i = 0; i < 11; i++) {

	    sleep(1);

 

	    double distance = sqrt(pow((list_x[i+1] - list_x[i]),2) + pow((list_y[i+1] - list_y[i]),2));

 

	    double value = atan2((list_y[i+1] - list_y[i]), (list_x[i+1] - list_x[i])) * (180 / 3.14);

            double theta = value - pre_theta;

 

            printf("main: value = %lf, theta = %lf, pre_theta = %lf\n", value, theta, pre_theta);

 

	    pre_theta = atan2((list_y[i+1] - list_y[i]), (list_x[i+1] - list_x[i])) * (180 / 3.14);

 

 

 

	    // get a parameters.

 

	    //double dRotation = atof(argv[1]);

 

	    double dRotation = theta;

 

	    float _dRatation = (float)((int)dRotation % 360);

 

	    //double dTranslation = atof(argv[2]);

 

	    double dTranslation = distance;

 

 

	

 

	    if(abs(_dRatation) > 180){

 

		  if(dRotation > 0) dRotation = -(360-_dRatation);

 

		  else dRotation = (360+_dRatation); }

 

	    else

 

		dRotation = _dRatation;

 

	

 

	    printf("main : dRotation = %lf\n", dRotation);

 

	    //initialTransformation = getCurrentTransformation();

    	    tf::Transform initialTransformation = getInitialTransformation();

 

 

 

	    //printf("execute Rotation\n");

 

	    doRotation(pub, initialTransformation, toRadian(dRotation), 0.1);

 

 

 

	    //printf("execute Translation\n");

 

	    doTranslation(pub, initialTransformation, list_x[i+1], list_y[i+1], dTranslation, 0.2);

 

    }

    geometry_msgs::Twist baseCmd;

    baseCmd.linear.x = 0.0;

    baseCmd.linear.y = 0.0;

 

    baseCmd.angular.z = 0.0;

 

    pub.publish(baseCmd);

 

 

    close(fd);

 

    return 0;

 

}