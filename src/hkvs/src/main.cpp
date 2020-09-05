#include <time.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include<memory>
#include <opencv2/ml/ml.hpp>
#include<iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>
#include <hkvs/square.h>


using namespace std;
using namespace cv;

ostringstream oss;
int num = 0;
Mat dealimage;
Mat src;
int k = 0;
Mat yangben_gray;
Mat yangben_thresh;
cv::Mat frame;

cv_bridge::CvImagePtr cv_ptr;
class ImageConverter
{
private:
    hkvs::square squared;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    //image_transport::Subscriber sub = it.subscribe( "/rrbot/camera1/image_raw", 1, imageCalllback );
    ros::Publisher squarepub = nh_.advertise<hkvs::square>("point2d", 1);
public:
    ImageConverter()
    : it_(nh_)
    {
        //设置订阅摄像机

        image_sub_ = it_.subscribe("/swivel/rrbot/camera1/image_raw", 1, &ImageConverter::imageCalllback, this);
    }

    ~ImageConverter(){
    }
    void imageCalllback(const sensor_msgs::ImageConstPtr& msg)
    {
            //ROS_INFO("Received \n");
            try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            //cv::imshow( "video", cv_bridge::toCvShare(msg, "bgr8")->image );
        }
        catch( cv_bridge::Exception& e )
        {
            ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str() );
            return;
        }
        frame=cv_ptr->image;
        image_process(frame);
    }
    void image_process(cv::Mat frame)
{
        clock_t start,finish;
        double totaltime;
        int hi = 0;
        Mat image,binary;
        int stateNum = 4;
        int measureNum = 2;
        frame.copyTo(binary);
        KalmanFilter KF(stateNum, measureNum, 0);
        //Mat processNoise(stateNum, 1, CV_32F);
        Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
        KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        //这里没有设置控制矩阵B，默认为零
        setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
        setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值

 

        imshow("Original", frame);
        
        //** convert to HSV
        Mat hsv;
        cvtColor(frame,hsv,COLOR_BGR2HSV);
        
        //** Find the BLACK Area
        Mat blkobj;
        Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
        //morphologyEx(hsv, hsv, MORPH_OPEN,element);
        //morphologyEx(hsv, hsv, MORPH_CLOSE,element);
        inRange(hsv,Scalar(0,0,0),Scalar(180,255,30),blkobj);

        imshow("BlackArea", blkobj);
                    vector<vector<Point>> contours;
            findContours(blkobj.clone(), contours, RETR_LIST, CHAIN_APPROX_NONE);
            Mat drawingImg = Mat::zeros(frame.size(), CV_8UC3);
            for (size_t i = 0; i < contours.size(); i++){
                if (contours.size() < 5) break;
                vector<Point> points;
                double area = contourArea(contours[i]);
                //cout<<"area"<<i<<" "<<area<<endl;
                if (area < 300 || 1000 < area) continue;
                cout<<1;
                drawContours(frame, contours, static_cast<int>(i), Scalar(0), 2);  
                points = contours[i];
                RotatedRect rrect = fitEllipse(points);
                cv::Point2f* vertices = new cv::Point2f[4];
                rrect.points(vertices);

                float aim = rrect.size.height/rrect.size.width;
                cout<<"aim="<<aim<<endl;
                if(aim > 0.8&& aim < 1.5){
                    cout<<2;
                    for (int j = 0; j < 4; j++)
                    {
                        cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),4);
				for (int j = 0; j < 4; j++){
        if(j==0){squared.zs_x=vertices[j].x;squared.zs_y=vertices[j].y;}
        if(j==1){squared.ys_x=vertices[j].x;squared.ys_y=vertices[j].y;}
        if(j==2){squared.yx_x=vertices[j].x;squared.yx_y=vertices[j].y;}
        if(j==3){squared.zx_x=vertices[j].x;squared.zx_y=vertices[j].y;}}
	cout<<"zuoshang"<<squared.zs_x<<".."<<squared.zs_y<<endl;
	cout<<"youshang"<<squared.ys_x<<".."<<squared.ys_y<<endl;
	cout<<"zuoxia"<<squared.zx_x<<".."<<squared.zx_y<<endl;
	cout<<"youxia"<<squared.yx_x<<".."<<squared.yx_y<<endl;

                    }
 		squared.square_num=1;
		squarepub.publish(squared); 
                    float middle = 100000;

                    for(size_t j = 1;j < contours.size();j++){

                        vector<Point> pointsA;
                        double area = contourArea(contours[j]);
                        if (area < 50 || 1e4 < area) continue;

                        pointsA = contours[j];

                        RotatedRect rrectA = fitEllipse(pointsA);

                        float aimA = rrectA.size.height/rrectA.size.width;
                        cout<<"aimA="<<aimA<<endl;
                        if(aimA > 3.0){
                        float distance = sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                                              (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));

                        if (middle > distance  )
                            middle = distance;
                        }
                    }
                    cout<<"distance: "<<middle<<endl;
                    if( middle > 10){      //这个距离也要根据实际情况调             
                       cout<<"center="<<rrect.center.x<<" "<<rrect.center.y<<endl;
                        cv::circle(binary,Point(rrect.center.x,rrect.center.y),15,cv::Scalar(0,0,255),4);
                        Mat prediction = KF.predict();
                        Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));
                        measurement.at<float>(0) = (float)rrect.center.x;
                        measurement.at<float>(1) = (float)rrect.center.y;
                        KF.correct(measurement);

                        circle(binary, predict_pt, 3, Scalar(34, 255, 255), -1);

                        rrect.center.x = (int)prediction.at<float>(0);
                        rrect.center.y = (int)prediction.at<float>(1);


                    }
                }
            }
            imshow("frame",binary); 
            waitKey(1);

            finish = clock();
            totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
            cout<<"Time whole"<<totaltime<<"秒"<<endl;

        
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_listener");
    ImageConverter ic;
    ros::spin();
    return 0;
}
