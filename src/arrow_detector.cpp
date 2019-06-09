#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <knu_ros_team4/arrowDetecter.h>
#include <iomanip>
#include <utility>

using namespace cv;
using namespace std;
///////////////////////////////////////////////////////////////////////////////
vector< pair<double,double> > square; //90 degree == 270 degree (0.1 radian)
vector< pair<double,double> > triangle; // 0.4 radian, 2
vector< pair<double,double> > triangle_large; // 0.8 radian, 1
ros::Publisher pub;
///////////////////////////////////////////////////////////////////////////////
//angle between pt1, pt2 and pt0 -> pt0 is middle
double angle( Point pt1, Point pt2, Point pt0 ) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
///////////////////////////////////////////////////////////////////////////////
bool isInVector(vector<pair<double,double> > v, Point value){
	for(int i=0;i<v.size();i++){
		if(v[i].first == value.x && v[i].second == value.y)
			return true;
	}
	return false;
}
///////////////////////////////////////////////////////////////////////////////
// find all od squares
void find_arrows( Mat& image, vector< vector< Point> >& arrows)
{
	// blur will enhance edge detection
	GaussianBlur(image, image, Size(5,5), 1 ,1);
	Mat gray0;
	cvtColor(image, gray0, CV_BGR2GRAY);
	Mat gray(image.size(), CV_8U);
	vector< vector< Point> > contours;
	gray = gray0 >= 100;
	// Find contours and store them in a list
	findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	// Test contours
	vector< Point> approx;
	
	
	vector< pair<double,double> > square_test;
	vector< pair<double,double> > triangle_test;
	vector< pair<double,double> > triangle_large_test;
	for (size_t i = 0; i < contours.size(); i++)
	{
	
		approxPolyDP( Mat(contours[i]), approx, arcLength( Mat(contours[i]), true)*0.02, true);
		
		// find arrow type!!	
		// && isContourConvex( Mat(approx))
		if (approx.size() == 7 && fabs(contourArea( Mat(approx))) > 5000 )
		{
			double maxCosine = 0;
			for (int j = 2; j <= 8; j++)//why 8? - last-first edge check
			{
				double cosine = fabs(angle(approx[j%7], approx[j-2], approx[j-1]));
				//register points
				if(cosine<0.28){ //not minus value
					//printf("cosine sq: %lf ",cosine); 	
					if(!isInVector(square_test,approx[j-1])) square_test.push_back(make_pair(approx[j-1].x,approx[j-1].y));	
				}
				else if(cosine>0.35 && cosine<0.58){//triangle small angles
					//printf("cosine tri: %lf ",cosine); 
					if(!isInVector(triangle_test,approx[j-1])) triangle_test.push_back(make_pair(approx[j-1].x,approx[j-1].y));
				}
				else if(cosine>0.78){// triangle large angle
					//printf("cosine tri large: %lf ",cosine); 
					if(!isInVector(triangle_large_test,approx[j-1])) triangle_large_test.push_back(make_pair(approx[j-1].x,approx[j-1].y));
				}
			}
			//printf("\n");
			//4 squre, 3 triangle angle == arrow

		}
		if (square_test.size()==4&& triangle_test.size() == 2 && triangle_large_test.size() == 1){ 
			//check arrow delicately
			arrows.push_back(approx);
			square_test.swap(square);
			triangle_test.swap(triangle);
			triangle_large_test.swap(triangle_large);
		}
		
		
		
	}
}
///////////////////////////////////////////////////////////////////////////////
// find largest one!!
void find_largest_arrow(const vector<vector <Point> >& arrows, vector<Point>& biggest_arrow) {
	if (!arrows.size()) {
		return;
	}
	int max_width = 0;
	int max_height = 0;
	int max_square_idx = 0;
	const int n_points = 4;
	for (size_t i = 0; i < arrows.size(); i++) {
		Rect rectangle = boundingRect(Mat(arrows[i]));
		if ((rectangle.width >= max_width) && (rectangle.height >= max_height)) {
			max_width = rectangle.width;
			max_height = rectangle.height;
			max_square_idx = i;
		}
	}
	biggest_arrow = arrows[max_square_idx];
}
///////////////////////////////////////////////////////////////////////////////
//return left:0, right:1
int whichSide(vector<Point>& arrow)
{
	//how about slide station??
	
	printf("\nsquare: ");
	for(int i=0;i<square.size();i++){
		printf("(%lf,%lf)",square[i].first,square[i].second);
	}
	printf("\ntriangle: ");
	for(int i=0;i<triangle.size();i++){
		printf("(%lf,%lf)",triangle[i].first,triangle[i].second);
	}
	printf("\ntriangle large: ");
	for(int i=0;i<triangle_large.size();i++){
		printf("(%lf,%lf)",triangle_large[i].first,triangle_large[i].second);
	}
	
	
	//large triangle's y value > any squre's y value -> left (opencv's w is below direction)
	//printf("tri large: %lf, squre first: %lf\n",triangle_large[0].first ,square[0].first);
	if(triangle_large[0].first > square[0].first){
		return 1;
	}else{
		return 0;
	}
	
}

///////////////////////////////////////////////////////////////////////////////
// A callback function. Executed eack time a new pose message arrives.
void poseMessageReceivedRGB(const sensor_msgs::ImageConstPtr& msg) {
	Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
	Mat img_origin = img.clone();
	vector< vector< Point> > arrows;
	find_arrows(img, arrows);
	vector<Point> largest_arrow;
	find_largest_arrow(arrows, largest_arrow);
	//draw largest arrow
	if(largest_arrow.size() >0 ) {
		printf("<<<<ARROW DETECTION>>>>\n");
		for (int i = 0; i < 7; i++) {
			line(img, largest_arrow[i], largest_arrow[(i+1)%7], Scalar(0, 0, 255), 3, CV_AA);
		}
	}
	//imshow("img_origin",img_origin);
	imshow("arrows", img);
	waitKey(1);//all time
	
	//check direction
	knu_ros_team4::arrowDetecter msgAD;
	
	if(largest_arrow.size() >0 ) {
		if(whichSide(largest_arrow)) {
			printf("\narrow direction is right\n");
			msgAD.intAD = 0;
		}
		else {
			printf("\narrow direction is left\n");
			msgAD.intAD = 1;
		}
		pub.publish(msgAD);
	}
	
	//clear
	square.clear();
	triangle.clear();
	
	return;
}
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "arrow_detector");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	// Create a subscriber object
	image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &poseMessageReceivedRGB, ros::VoidPtr(), image_transport::TransportHints("compressed"));
	
	pub = nh.advertise<knu_ros_team4::arrowDetecter>("arrowDetecter",100);
	// Let ROS take over
	ros::spin();
	return 0;
}
