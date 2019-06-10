// ?�본 코드  https://github.com/georgesung/road_lane_line_detection/blob/master/lane_lines.py
// ?�정 - webnautes
//
// ?�요???�이브러�?// OpenCV 3.x  http://opencv.org/releases.html

// ?�치 방법 http://webnautes.tistory.com/1186

//
// GSL - GNU Scientific Library https://www.gnu.org/software/gsl/
// ?�치 방법 sudo apt-get install libgsl-dev
//
// 컴파??

// g++ main.cpp -o main $(pkg-config opencv --libs --cflags) -lgsl -lcblas

//
// ?�스???�영???�운로드 

// https://github.com/georgesung/road_lane_line_detection

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc.hpp>
#include <gsl/gsl_fit.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <iostream>  
#include <cv_bridge/cv_bridge.h>
#include <knu_ros_team4/arrowDetecter.h>



using namespace cv;
using namespace std;

ros::Publisher pub;
ros::Subscriber subAD;
geometry_msgs::Twist baseCmd;
boost::mutex mutex;


//Hough Transform ?�라미터
float rho = 2; // distance resolution in pixels of the Hough grid
float theta = 1 * CV_PI / 180; // angular resolution in radians of the Hough grid
float hough_threshold = 15;    // minimum number of votes(intersections in Hough grid cell)
float minLineLength = 10; //minimum number of pixels making up a line
float maxLineGap = 20;   //maximum gap in pixels between connectable line segments


//Region - of - interest vertices, 관???�역 범위 계산???�용 
//We want a trapezoid shape, with bottom edge at the bottom of the image
float trap_bottom_width = 0.85;  // width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 0.07;     // ditto for top edge of trapezoid
float trap_height = 0.4;         // height of the trapezoid expressed as percentage of image height

int camera_width = 640;
int camera_height = 480;
int pre_center_x;

//차선 ?�깔 범위 
//SCALAR LOWER_WHITE = SCALAR(200, 200, 200); //?�색 차선 (RGB)
//SCALAR UPPER_WHITE = SCALAR(255, 255, 255);
//SCALAR LOWER_YELLOW = SCALAR(10, 100, 100); //?��???차선 (HSV)
//SCALAR UPPER_YELLOW = SCALAR(40, 255, 255);

Scalar lower_white = Scalar(0, 0, 0); //?�색 차선 (RGB)
Scalar upper_white = Scalar(10, 100, 100);
Scalar lower_yellow = Scalar(0, 0, 0); //?��???차선 (HSV)
Scalar upper_yellow = Scalar(10, 100, 100);

Mat img, img_masked, img_mask;
Mat img_bgr, img_gray, img_edges, img_hough, img_annotated;   
 
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
   if(increment_ratio > 1) {
	increment_ratio = 0.9;
   }
   if(center_x1 > camera_width / 2 + (camera_width / 20)){
      cout << "Turn Left" << endl;
      baseCmd.angular.z = 0.1 * (increment_ratio + 1);

      if(baseCmd.angular.z < 0){
		baseCmd.angular.z *= -1;
      }

      baseCmd.linear.x = 0.03;

   }
   else if(center_x1 < camera_width / 2 - (camera_width / 20)){
       cout << "Turn Right" << endl;
       baseCmd.angular.z = -0.1 * (increment_ratio + 1);
       if(baseCmd.angular.z > 0){
		baseCmd.angular.z *= -1;
       }
      baseCmd.linear.x = 0.03;
   }else{
      baseCmd.angular.z = 0;
      baseCmd.linear.x = 0.03;
   }

   if(baseCmd.angular.z > 0.4) {
     baseCmd.angular.z = 0.3;
   }

   cout << "--------angular speed = " << baseCmd.angular.z << endl;
   pub.publish(baseCmd);
}


void draw_line(Mat &img_line, vector<Vec4i> lines)
{
   cout << " in draw_line & lines.size : " << lines.size() << endl;
   if (lines.size() == 0) return;

   /*
   NOTE : this is the function you might want to use as a starting point once you want to
   average / extrapolate the line segments you detect to map out the full
   extent of the lane(going from the result shown in raw - lines - example.mp4
   to that shown in P1_example.mp4).

   Think about things like separating line segments by their
   slope((y2 - y1) / (x2 - x1)) to decide which segments are part of the left
   line vs.the right line.Then, you can average the position of each of
   the lines and extrapolate to the top and bottom of the lane.

   This function draws `lines` with `color` and `thickness`.
   Lines are drawn on the image inplace(mutates the image).
   If you want to make the lines semi - transparent, think about combining
   this function with the weighted_img() function below
   */

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

      //printf("# best fit: Y = %g + %g X\n", c0, c1);

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

      //printf("# best fit: Y = %g + %g X\n", c0, c1);

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
   } else if((left_x1 < 0 || left_x2 < 0) && (right_x1 > 0 && right_x2 > 0 && right_x1 < camera_width && right_x2 < camera_width)) { // ���ʼ� �Ⱥ��϶� : ��ȸ
      cout << "Calculate : Turn Left" << endl;
      center_x1 = (right_x1 + camera_width / 2) / 2;
   } else if((right_x1 < 0 || right_x2 < 0) && (left_x1 > 0 && left_x2 > 0 && left_x1 < camera_width && left_x2 < camera_width)) { // �����ʼ� �Ⱥ��϶�:��ȸ
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

    
   //2. 미리 ?�해???�색, ?��???범위 ?�에 ?�는 부분만 차선?�보�??�로 ?�?�함 
   Mat img_filtered;
   filter_colors(img_bgr, img_filtered);

   //3. 그레?�스케???�상?�로 변?�하???��? ?�분??추출
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


   //4. 차선 검출할 ?�역???�한??진행방향 바닥??존재?�는 차선?�로 ?�정)
   img_edges = region_of_interest(img_edges, points);


   UMat uImage_edges;
   img_edges.copyTo(uImage_edges);

   //5. 직선 ?�분??추출(�?직선???�작좌표?� ?�좌?��? 계산??
   vector<Vec4i> lines;
   HoughLinesP(uImage_edges, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);




   //6. 5번에??추출??직선?�분?�로부??좌우 차선???�을 가?�성?�는 직선?�만 ?�로 뽑아??   //좌우 각각 ?�나??직선??계산??(Linear Least-Squares Fitting)
   Mat img_line = Mat::zeros(img_bgr.rows, img_bgr.cols, CV_8UC3);
   draw_line(img_line, lines);


   //7. ?�본 ?�상??6번의 직선??같이 보여�?
   addWeighted(img_bgr, 0.8, img_line, 1.0, 0.0, img_annotated);
    
   

   //9. 결과�??�면??보여�?
   
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
   //subAD = nh.subscribe("arrowDetecter",100,&arrowMessage);
   image_transport::Subscriber sub = it.subscribe("/raspicam_node/image", 100, &poseMessageReceived, ros::VoidPtr(), image_transport::TransportHints("compressed"));

  while(ros::ok()){
    cout << "main" << endl;
    baseCmd.linear.x = 0.01;
    pub.publish(baseCmd);
     ros::spin();
  }




   return 0;
}
