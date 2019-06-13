#include <vector>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc.hpp>
#include <gsl/gsl_fit.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <iostream>  
#include <cv_bridge/cv_bridge.h>
#include <knu_ros_team4/arrowDetecter.h>

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

using namespace cv;
using namespace std;

ros::Publisher pub;
ros::Subscriber subAD;
ros::Subscriber subOdom;
ros::Subscriber subScan;
geometry_msgs::Twist baseCmd;
sensor_msgs::LaserScan g_scan;
nav_msgs::Odometry g_odom;
boost::mutex mutex;
boost::mutex mutex_scan;
boost::mutex mutex_odom;

// Flag
int OBSTACLE_FLAG= 0;

//Hough Transform
float rho = 2; // distance resolution in pixels of the Hough grid
float theta = 1 * CV_PI / 180; // angular resolution in radians of the Hough grid
float hough_threshold = 15;    // minimum number of votes(intersections in Hough grid cell)
float minLineLength = 10; //minimum number of pixels making up a line
float maxLineGap = 20;   //maximum gap in pixels between connectable line segments

float delta = 0.349; // 20 degree

//Region - of - interest vertices
//We want a trapezoid shape, with bottom edge at the bottom of the image
float trap_bottom_width = 0.85;  // width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 0.07;     // ditto for top edge of trapezoid
float trap_height = 0.4;         // height of the trapezoid expressed as percentage of image height

int camera_width = 900;
int camera_height = 600;
int pre_center_x;
int arrow = -1;
//SCALAR LOWER_WHITE = SCALAR(200, 200, 200); //(RGB)
//SCALAR UPPER_WHITE = SCALAR(255, 255, 255);
//SCALAR LOWER_YELLOW = SCALAR(10, 100, 100); //(HSV)
//SCALAR UPPER_YELLOW = SCALAR(40, 255, 255);

Scalar lower_white = Scalar(0, 0, 0); //(RGB)
Scalar upper_white = Scalar(10, 100, 100);
Scalar lower_yellow = Scalar(0, 0, 0); //(HSV)
Scalar upper_yellow = Scalar(10, 100, 100);

Mat img, img_masked, img_mask;
Mat img_bgr, img_gray, img_edges, img_hough, img_annotated;   

template<typename T>
inline bool isnan(T value)
{
    return value != value;
}

void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    mutex_odom.lock(); {
        g_odom = msg;
    } mutex_odom.unlock();
}

void
transform(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
    Vec3d newPt;
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    int nRangeSize = (int)laserScanXY.size();
    // rotation matrix and translation matrix multiply R*T*Position
    for(int i=0; i<nRangeSize; i++) {
        newPt[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
        newPt[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
        newPt[2];
        laserScanXY[i] = newPt;
    }
}

void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
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


void
convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy)
{
    // 이동 저장
    xyz[0] = odom.pose.pose.position.x;
    xyz[1] = odom.pose.pose.position.y;
    xyz[2] = odom.pose.pose.position.z;

    // 회전 저장
    tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);
}


void
distanceCheck(vector<Vec3d> &laserScanXY, double theta, double delta, double dMaxDist){
    int nRangeSize = static_cast<int>(laserScanXY.size());
    double dAngle;
    double distance;
    double obstacleMean = 0;
    double obstacleCnt = 0;
    int NearDistanceCheck = 0;
    for(int i = 0; i < nRangeSize; i++){
        dAngle = atan2(laserScanXY[i][1] - g_odom.pose.pose.position.y,
                laserScanXY[i][0] - g_odom.pose.pose.position.x);
        if(dAngle < theta + delta && dAngle > theta - delta){
            distance = sqrt(pow(laserScanXY[i][0] - g_odom.pose.pose.position.x,2)
                    + pow(laserScanXY[i][1] - g_odom.pose.pose.position.y,2));
            //cout << "Near distance: " << distance << endl;
            if(distance < 0.43){
                NearDistanceCheck = 1;
                obstacleMean += distance;
                obstacleCnt++;
            }

        }
    }
    if(NearDistanceCheck == 1){
        obstacleMean /= obstacleCnt;
        cout << "장애물발견" << ", obstacle mean distance: " << obstacleMean << endl;
        OBSTACLE_FLAG = 1;
    }
    else
        OBSTACLE_FLAG = 0;
}

void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    mutex_scan.lock(); {
        g_scan = msg;
    } mutex_scan.unlock();
    // 이동 및 회전 정보
    Vec3d xyz, rpy;
    // LRF scan 정보
    vector<Vec3d> laserScanXY;

     // Mat distance for grid
    const double dGridMaxDist = 4.0;

    convertOdom2XYZRPY(g_odom,xyz,rpy);
    convertScan2XYZs(g_scan,laserScanXY);

    transform(laserScanXY,xyz[0],xyz[1],rpy[2]);
    distanceCheck(laserScanXY,rpy[2],static_cast<double>(delta),dGridMaxDist);
}


Mat region_of_interest(Mat img_edges, Point *points)
{
   /*
   Applies an image mask.

   Only keeps the region of the image defined by the polygon
   formed from `vertices`. The rest of the image is set to black.
   */

   Mat img_mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);


   Scalar ignore_mask_color = Scalar(255, 255, 255);
   const Point* ppt[1] = { points };
   int npt[] = { 4 };


   //filling pixels inside the polygon defined by "vertices" with the fill color
   fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);


   //returning the image only where mask pixels are nonzero
   Mat img_masked;
   bitwise_and(img_edges, img_mask, img_masked);


   return img_masked;
}




void filter_colors(Mat _img_bgr, Mat &img_filtered)
{
   // Filter the image to include only yellow and white pixels
   UMat img_bgr1;
   _img_bgr.copyTo(img_bgr1);
   UMat img_hsv, img_combine;
   UMat white_mask, white_image;
   UMat yellow_mask, yellow_image;


   //Filter white pixels
   inRange(img_bgr1, lower_white, upper_white, white_mask);
   bitwise_and(img_bgr1, img_bgr1, white_image, white_mask);


   //Filter yellow pixels( Hue 30 )
   cvtColor(img_bgr1, img_hsv, COLOR_BGR2HSV);


   inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
   bitwise_and(img_bgr1, img_bgr1, yellow_image, yellow_mask);


   //Combine the two above images
   addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_combine);


   img_combine.copyTo(img_filtered);
   
   img_bgr1.release();
   img_hsv.release();
   img_combine.release();
   white_mask.release();
   white_image.release();
   yellow_mask.release();
   yellow_image.release();
}

void move_robot(int center_x1, float left_slope, float right_slope) {
   float increment_ratio = fabs(center_x1 / ((camera_width / 2) - 1)); // 90% or 110% -> result : 0.1

   cout << "centerx : " << center_x1 << ", left_slope : " << left_slope << ", right_slpe : "
<< right_slope << endl;
	
   // 계산한 직선의 기울기의 비율이 너무크면 최대값으로 고
   if(increment_ratio > 1) {
	increment_ratio = 0.9;
   }

   // 두 차선의 중심이 오른쪽으로 가면 왼쪽으로 회
   if(center_x1 > camera_width / 2 + (camera_width / 10)){
      cout << "Turn Left" << endl;
	baseCmd.linear.x = 0.06;
	baseCmd.angular.z = 0;
       for(int i = 0; i < 100; i++) {
	   pub.publish(baseCmd);
	}
      baseCmd.angular.z = 0.2 * (increment_ratio + 1);

      if(baseCmd.angular.z < 0){
		baseCmd.angular.z *= -1;
      }

   }
   // 두 차선의 중심이 왼쪽으로 가면 오른쪽으로 회전
   else if(center_x1 < camera_width / 2 - (camera_width / 10)){
       cout << "Turn Right" << endl;
	baseCmd.linear.x = 0.06;
	baseCmd.angular.z = 0;
       for(int i = 0; i < 100; i++) {
	   pub.publish(baseCmd);
	}
       baseCmd.angular.z = -0.2 * (increment_ratio + 1);
       if(baseCmd.angular.z > 0){
		baseCmd.angular.z *= -1;
       }
      
   }else{
      baseCmd.angular.z = 0;
      baseCmd.linear.x = 0.06;
   }
   // 회전속도가 혹시 너무 높아지면 한계값으로 설
   if(baseCmd.angular.z > 0.4) {
     baseCmd.angular.z = 0.3;
   }

   cout << "--------angular speed = " << baseCmd.angular.z << endl;
	   
   // 수진이 코드 합친것
   // Arrow 발견되면 flag값을 1로 변경후 회전속도 설
   int flag = 0;
   if(arrow == 1) { // right arrow
	baseCmd.angular.z = -0.75;
	flag = 1;
   } else if(arrow == 0) { //left arrow
	baseCmd.angular.z = 0.75;
	flag = 1;
   }
   
   // 정우형 코드 부분
   if(OBSTACLE_FLAG == 1) {
	// 앞의 차량 발견되면?
       baseCmd.linear.x = g_odom.twist.twist.linear.x/2;
   } else if(OBSTACLE_FLAG == 0) {
	// 그냥 정상주
   }

   pub.publish(baseCmd);
}


void draw_line(Mat &img_line, vector<Vec4i> lines)
{
   cout << " in draw_line & lines.size : " << lines.size() << endl;
   if (lines.size() == 0) return;

   // In case of error, don't draw the line(s)
   bool draw_right = true;
   bool draw_left = true;
   int width = img_line.cols;
   int height = img_line.rows;


   //Find slopes of all lines
   //But only care about lines where abs(slope) > slope_threshold
   float slope_threshold = 0.5;
   vector<float> slopes;
   vector<Vec4i> new_lines;

   for (int i = 0; i < lines.size(); i++)
   {
      Vec4i line = lines[i];
      int x1 = line[0];
      int y1 = line[1];
      int x2 = line[2];
      int y2 = line[3];


      float slope;
      //Calculate slope
      if (x2 - x1 == 0) //corner case, avoiding division by 0
         slope = 999.0; //practically infinite slope
      else
         slope = (y2 - y1) / (float)(x2 - x1);


      //Filter lines based on slope
      if (abs(slope) > slope_threshold) {
         slopes.push_back(slope);
         new_lines.push_back(line);
      }
   }



   // Split lines into right_lines and left_lines, representing the right and left lane lines
   // Right / left lane lines must have positive / negative slope, and be on the right / left half of the image
   vector<Vec4i> right_lines;
   vector<Vec4i> left_lines;

   for (int i = 0; i < new_lines.size(); i++)
   {

      Vec4i line = new_lines[i];
      float slope = slopes[i];

      int x1 = line[0];
      int y1 = line[1];
      int x2 = line[2];
      int y2 = line[3];


      float cx = width * 0.5; //x coordinate of center of image

      if (slope > 0 && x1 > cx && x2 > cx)
         right_lines.push_back(line);
      else if (slope < 0 && x1 < cx && x2 < cx)
         left_lines.push_back(line);
   }


   //Run linear regression to find best fit line for right and left lane lines
   //Right lane lines
   double right_lines_x[1000];
   double right_lines_y[1000];
   float right_m, right_b;


   int right_index = 0;
   for (int i = 0; i < right_lines.size(); i++) {

      Vec4i line = right_lines[i];
      int x1 = line[0];
      int y1 = line[1];
      int x2 = line[2];
      int y2 = line[3];

      right_lines_x[right_index] = x1;
      right_lines_y[right_index] = y1;
      right_index++;
      right_lines_x[right_index] = x2;
      right_lines_y[right_index] = y2;
      right_index++;
   }


   if (right_index > 0) {

      double c0, c1, cov00, cov01, cov11, sumsq;
      gsl_fit_linear(right_lines_x, 1, right_lines_y, 1, right_index,
         &c0, &c1, &cov00, &cov01, &cov11, &sumsq);

      right_m = c1;
      right_b = c0;
   }
   else {
      //right_m = right_b = 1;
      right_b = 9999;
      right_m = 1;

      draw_right = false;
   }



   // Left lane lines
   double left_lines_x[1000];
   double left_lines_y[1000];
   float left_m, left_b;

   int left_index = 0;
   for (int i = 0; i < left_lines.size(); i++) {

      Vec4i line = left_lines[i];
      int x1 = line[0];
      int y1 = line[1];
      int x2 = line[2];
      int y2 = line[3];

      left_lines_x[left_index] = x1;
      left_lines_y[left_index] = y1;
      left_index++;
      left_lines_x[left_index] = x2;
      left_lines_y[left_index] = y2;
      left_index++;
   }


   if (left_index > 0) {
      double c0, c1, cov00, cov01, cov11, sumsq;
      gsl_fit_linear(left_lines_x, 1, left_lines_y, 1, left_index,
         &c0, &c1, &cov00, &cov01, &cov11, &sumsq);
	   
      left_m = c1;
      left_b = c0;
   }
   else {
      //left_m = left_b = 1;
      left_b = 9999;
      left_m = 1;
      draw_left = false;
   }



   //Find 2 end points for right and left lines, used for drawing the line
   //y = m*x + b--> x = (y - b) / m
   int y1 = height;
   int y2 = height * (1 - trap_height);

   float right_x1 = (y1 - right_b) / right_m;
   float right_x2 = (y2 - right_b) / right_m;

   float left_x1 = (y1 - left_b) / left_m;
   float left_x2 = (y2 - left_b) / left_m;
   
   //Convert calculated end points from float to int
   y1 = int(y1);
   y2 = int(y2);
   right_x1 = int(right_x1);
   right_x2 = int(right_x2);
   left_x1 = int(left_x1);
   left_x2 = int(left_x2);
   
   if(left_x1 > left_x2) {
     int temp = left_x1;
     left_x1 = left_x2;
     left_x2 = temp;
   }
   if(right_x1 < right_x2) {
     int temp = right_x1;
     right_x1 = right_x2;
     right_x2 = temp;
   }

   float left_slope = -1;
   float right_slope = -1;
   
   if(left_x1 > 0) {
	left_slope = (y2 - y1) / (float)(left_x2 - left_x1);
	}
   if(right_x1 > 0){
	right_slope = (y2 - y1) / (float)(right_x2 - right_x1);
	}

   cout << "left_x1=" << left_x1 << ", left_x2=" << left_x2 << ", right_x1=" << right_x1 << ", right_x2=" << right_x2 << endl;
   
   int center_x1;

   if(left_x1 > 0 && left_x2 > 0 && right_x1 > 0 && right_x2 > 0 && left_x1 < camera_width && left_x2 < camera_width && right_x1 < camera_width && right_x2 < camera_width) {
      center_x1 = (right_x1 + left_x1) / 2;
   } else if((left_x1 < 0 || left_x2 < 0) && (right_x1 > 0 && right_x2 > 0 && right_x1 < camera_width && right_x2 < camera_width)) { // ¿ÞÂÊ¼± ¾Èº¸ÀÏ¶§ : ÁÂÈ¸
      cout << "Calculate : Turn Left" << endl;
      center_x1 = (right_x1 + camera_width / 2) / 2;
   } else if((right_x1 < 0 || right_x2 < 0) && (left_x1 > 0 && left_x2 > 0 && left_x1 < camera_width && left_x2 < camera_width)) { // ¿À¸¥ÂÊ¼± ¾Èº¸ÀÏ¶§:¿ìÈ¸
      cout << "Calculate : Turn Right" << endl;
      center_x1 = (left_x1 + camera_width / 2) / 2;
   } else {
      cout << "Calculate : Except!!" << endl;
      center_x1 = pre_center_x;
   }

   move_robot(center_x1, left_slope, right_slope);
   pre_center_x = center_x1;   
   
   //Draw the right and left lines on image
   
   if (draw_right)
      line(img_line, Point(right_x1, y1), Point(right_x2, y2), Scalar(255, 0, 0), 10);
   if (draw_left)
      line(img_line, Point(left_x1, y1), Point(left_x2, y2), Scalar(255, 0, 0), 10);
 

    left_x1 = left_x2 = right_x1 = right_x2 = -9999;
}


void calculate(){

   cout << "i'm calculate" << endl;
  
   char buf[256];
   

   img_bgr = img.clone();

   int width = img_bgr.size().width;
   int height = img_bgr.size().height;

    
   //2. ë¯¸ë¦¬ ?•í•´???°ìƒ‰, ?¸ë???ë²”ìœ„ ?´ì— ?ˆëŠ” ë¶€ë¶„ë§Œ ì°¨ì„ ?„ë³´ë¡??°ë¡œ ?€?¥í•¨ 
   Mat img_filtered;
   filter_colors(img_bgr, img_filtered);

   //3. ê·¸ë ˆ?´ìŠ¤ì¼€???ìƒ?¼ë¡œ ë³€?˜í•˜???ì? ?±ë¶„??ì¶”ì¶œ
   cvtColor(img_filtered, img_gray, COLOR_BGR2GRAY);
   GaussianBlur(img_gray, img_gray, Size(3, 3), 0, 0);
   Canny(img_gray, img_edges, 50, 150);



   width = img_filtered.cols;
   height = img_filtered.rows;


   Point points[4];
   points[0] = Point((width * (1 - trap_bottom_width)) / 2, height);
   points[1] = Point((width * (1 - trap_top_width)) / 2, height - height * trap_height);
   points[2] = Point(width - (width * (1 - trap_top_width)) / 2, height - height * trap_height);
   points[3] = Point(width - (width * (1 - trap_bottom_width)) / 2, height);


   //4. ì°¨ì„  ê²€ì¶œí•  ?ì—­???œí•œ??ì§„í–‰ë°©í–¥ ë°”ë‹¥??ì¡´ìž¬?˜ëŠ” ì°¨ì„ ?¼ë¡œ ?œì •)
   img_edges = region_of_interest(img_edges, points);


   UMat uImage_edges;
   img_edges.copyTo(uImage_edges);

   //5. ì§ì„  ?±ë¶„??ì¶”ì¶œ(ê°?ì§ì„ ???œìž‘ì¢Œí‘œ?€ ?ì¢Œ?œë? ê³„ì‚°??
   vector<Vec4i> lines;
   HoughLinesP(uImage_edges, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);




   //6. 5ë²ˆì—??ì¶”ì¶œ??ì§ì„ ?±ë¶„?¼ë¡œë¶€??ì¢Œìš° ì°¨ì„ ???ˆì„ ê°€?¥ì„±?ˆëŠ” ì§ì„ ?¤ë§Œ ?°ë¡œ ë½‘ì•„??   //ì¢Œìš° ê°ê° ?˜ë‚˜??ì§ì„ ??ê³„ì‚°??(Linear Least-Squares Fitting)
   Mat img_line = Mat::zeros(img_bgr.rows, img_bgr.cols, CV_8UC3);
   draw_line(img_line, lines);


   //7. ?ë³¸ ?ìƒ??6ë²ˆì˜ ì§ì„ ??ê°™ì´ ë³´ì—¬ì¤?
   addWeighted(img_bgr, 0.8, img_line, 1.0, 0.0, img_annotated);
    
   

   //9. ê²°ê³¼ë¥??”ë©´??ë³´ì—¬ì¤?
   
   Mat img_result;
   resize(img_annotated, img_annotated, Size(width*0.7, height*0.7));
   resize(img_edges, img_edges, Size(width*0.7, height*0.7));
   cvtColor(img_edges, img_edges, COLOR_GRAY2BGR);
   hconcat(img_edges, img_annotated, img_result);
   imshow("video", img_result);

   waitKey(30); 
  
}

void
arrowMessage(const knu_ros_team4::arrowDetecter &msg){
	ROS_INFO("Arrow Detecter : %d", msg.intAD);
	arrow = msg.intAD;
}


void poseMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
  mutex.lock();{
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    
  }mutex.unlock();
  
    cout << "in callback" << endl;
    calculate();
  
}
int main(int argc, char** argv)
{
   ros::init(argc, argv, "lane_driving");
   ros::NodeHandle nh, nhp;
   image_transport::ImageTransport it(nh);


   pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
   subAD = nh.subscribe("arrowDetecter",100,&arrowMessage);
   subScan = nh.subscribe("/scan",10,&scanMsgCallback);
   subOdom = nh.subscribe("/odom",100,&odomMsgCallback);
   image_transport::Subscriber sub = it.subscribe("/raspicam_node/image", 100, &poseMessageReceived, ros::VoidPtr(), image_transport::TransportHints("compressed"));


  while(ros::ok()){
    cout << "main" << endl;
    baseCmd.linear.x = 0.01;
    pub.publish(baseCmd);
     ros::spin();
  }




   return 0;
}
