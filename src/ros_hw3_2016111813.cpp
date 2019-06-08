#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>



using namespace cv;
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
float pre_dAngleTurned;
int deltaDegree = 10;
double averageDelta;
double limitDistance = 0.5;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A template method to check 'nan'
template<typename T>
inline bool isnan(T value)
{
    return value != value;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void odomMsgCallback(const nav_msgs::Odometry &msg)
{
    mutex[0].lock(); {
        g_odom = msg;
    } mutex[0].unlock();
    ROS_INFO("<<CURRENT POSITION>>(%lf,%lf)",msg.pose.pose.position.x,msg.pose.pose.position.y);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    mutex[1].lock(); {
        g_scan = msg;
    } mutex[1].unlock();
    double sum = 0.0;
    int length =0;
    double tooFar = limitDistance*4;
    //if some obtacle to far, then discard them.
    for(int i=0;i<=deltaDegree;i++)
    {
    	double dRange = msg.ranges[i];
    	if(!isnan(dRange) && !isinf(dRange) && (dRange<tooFar)){
    		sum +=dRange;
    		length++;
    	}
    }
    for(int i=(360-deltaDegree);i<360;i++)
    {
    	double dRange = msg.ranges[i];
    	if(!isnan(dRange) && !isinf(dRange) && (dRange<tooFar)){
    		sum +=dRange;
    		length++;
    	}
    }
    if(length!=0){
        averageDelta = sum/(2*(double)length);
        //ROS_INFO("LENGTH IS %d",length);
    }
    else{
    	averageDelta=0;
   	}
    	
    ROS_INFO(">>AVERAGE DISTANCE<< ( %lf )",averageDelta); 
    
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initGrid(Mat &display, int nImageSize)
{
    const int nImageHalfSize = nImageSize/2;
    const int nAxisSize = nImageSize/16;
    const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
    display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
    int nRangeSize = (int)lrfScan.ranges.size();
    XYZs.clear();
    XYZs.resize(nRangeSize);

    for(int i=0; i<nRangeSize; i++) {
        double dRange = lrfScan.ranges[i];

        if(isnan(dRange)) {
            XYZs[i] = Vec3d(0., 0., 0.);
        } else {
            double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
            XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++) {
        int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
            display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tf::Transform getCurrentTransformation(void)
{
    tf::Transform transformation;

    nav_msgs::Odometry odom;

    mutex[0].lock(); {
        odom = g_odom;
    } mutex[0].unlock();

    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    return transformation;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tf::Transform getInitialTransformation(void)
{
    tf::Transform transformation;

    ros::Rate loopRate(1000.0);

    while(ros::ok()) {
        ros::spinOnce();

        transformation = getCurrentTransformation();

        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }

    return transformation;
}
///////////////////////////////////////////////
void drawScan(Mat &display)
{

    sensor_msgs::LaserScan scan;
	vector<Vec3d> laserScanXY;
    const double dGridMaxDist = 4.5;
	
	ros::spinOnce();
	mutex[1].lock(); {
        scan = g_scan;
    } mutex[1].unlock();

    convertScan2XYZs(scan, laserScanXY);
    
	initGrid(display,400);
	drawLRFScan(display, laserScanXY, dGridMaxDist);
	
	imshow("ROS-HW3-2016111813", display);
	int nKey = waitKey(1) % 255; //1->no delay no key input

	return;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed,Mat &display)
{
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;

    if(dRotation < 0.) {
        baseCmd.angular.z = -dRotationSpeed;
    } else {
        baseCmd.angular.z = dRotationSpeed;
    }

    bool bDone = false;
    ros::Rate loopRate(1000.0);



    while(ros::ok() && !bDone) {

        ros::spinOnce();
        //opencv
    	drawScan(display);

        tf::Transform currentTransformation = getCurrentTransformation();

        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        tf::Quaternion rotationQuat = relativeTransformation.getRotation();


      
         double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));

    	//ROS_INFO("ROTATION:%lf, HA: %lf",dRotation,dAngleTurned);
	//|| (abs(pre_dAngleTurned - dRotation) <  abs(dAngleTurned - dRotation)) || (dRotation == 0)
    if( fabs(dAngleTurned) > fabs(dRotation) ) 
	{
            bDone = true;
            break;
        } else {
	    pre_dAngleTurned = dAngleTurned;
            pubTeleop.publish(baseCmd);
            loopRate.sleep();
        }
    }

    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "ros_hw3_2016111813");
    ros::NodeHandle nh1,nh2,nhp;

    // Create subscriber objects
    ros::Subscriber subOdom = nh1.subscribe("/odom", 100, &odomMsgCallback);
    ros::Subscriber subScan = nh2.subscribe("/scan", 10, &scanMsgCallback);
    ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	
	//opencv
	Mat display;
    initGrid(display, 400);

    //move straight
    geometry_msgs::Twist initMove;
    initMove.linear.x = 0.1;
    srand(time(NULL));
    
    //check obstacle
    sleep(1);
    double theta = 5.0;
    int random = rand()%2;
    if(random)
    	theta = -theta;
    while(ros::ok())
    {
    	ros::spinOnce();
    	pub.publish(initMove);
    	drawScan(display);
    	
    	if(averageDelta<=limitDistance && averageDelta>0){
    		geometry_msgs::Twist currMove;
    		ROS_INFO("OBTACLE!!!!!! ");
    		
    		//stop
    		currMove.linear.x = 0.0;
    		pub.publish(currMove);
    		
    		//rotation
    		tf::Transform initialTransformation = getInitialTransformation();
    		doRotation(pub, initialTransformation, toRadian(theta), 0.1,display);
    		
    		//move
    		currMove.linear.x = 0.1;
    		pub.publish(currMove);
    		
    	}else if(averageDelta>(2*limitDistance) || (averageDelta==0)){
    		//no obtacles -> change theta
    		//if not 2*limitDistance, too lausy
    		random = rand()%2;
    		if(random)
    			theta = -theta;
    		}
    	
    
    }
    
    
    
    
    return 0;
}

