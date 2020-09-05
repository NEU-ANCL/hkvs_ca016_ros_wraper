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
using namespace ml;

ostringstream oss;
int num = 0;
Mat dealimage;
Mat src;
int k = 0;
Mat yangben_gray;
Mat yangben_thresh;
hkvs::square squared;







Mat my_resize(const Mat &img, int width, int height)
{
    // 1, 定义缩放后的图像大小，行列缩放比例
    Mat output = Mat::zeros(Size(width, height), CV_8UC1);
    float width_scale = (float)img.cols / width;     // 列缩放比例，相对于算法前面讲的k1
    float height_scale = (float)img.rows / height;   // 行缩放比例，即k2

    // 2, 采样
    for (int i = 0; i < height; i++)  // 注意i,j的范围, i < height * img.rows / height;
    {
        for (int j = 0; j < width; j++)
        {
            output.at<uchar>(i, j) = img.at<uchar>(round(i * height_scale), round(j * width_scale));
        }
    }

    return output;

}
int main(int argc, char** argv)
{
        ros::init(argc, argv, "image_listener");
	    ros::NodeHandle nh_;
        ros::Publisher squarepub = nh_.advertise<hkvs::square>("point2d", 1);
        clock_t start,finish;
        double totaltime, heights[16];
        int hi = 0;
        VideoCapture capture("/home/tyj/5.avi");
        //VideoCapture capture(1);
        Mat frame, binary;
        RotatedRect RA[16], R[16];
        int stateNum = 4;//状态值4×1向量(x,y,△x,△y)
        int measureNum = 2;//测量值2×1向量(x,y)
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

        ros::Rate loop_rate(10);
        while(ros::ok())  
         {
            //要循环执行的代码
            cout<<"fff"<<endl;
            


            //发布消息给定位
            //把pub啥的粘贴进来

         ros::spinOnce();
         loop_rate.sleep();
        }

        for(;;)
         {
            double MaxValue=0;
            Mat grayImg;
            Mat binBrightImg;
            vector<Mat> channels;
            start = clock();
            capture>> frame;
            if(frame.empty())//如果某帧为空则退出循环
               break;
            frame.copyTo(binary);
            Mat HSVImg;
            Mat image;
            cvtColor(frame,HSVImg,COLOR_BGR2HSV);
            split(HSVImg, channels);
            minMaxLoc(channels[2], 0, &MaxValue, 0, 0);
            //cout << "单通道图像最大值： " << MaxValue << endl;
            threshold(channels[2], channels[2], MaxValue*0.98, 255, THRESH_BINARY);
            Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
            medianBlur(channels[2], channels[2], 3);
            morphologyEx(channels[2], channels[2], MORPH_DILATE, element, Point(-1, -1), 1);
            // imshow("V通道二值图", channels[2]); waitKey();
            HSVImg.copyTo(image, channels[2]);
            int BLowH = 80;
            int BHighH = 150;
            int BLowS = 60;
            int BHighS = 255;
            int BLowV = 100;
            int BHighV = 255;
            inRange(image, Scalar(BLowH, BLowS, BLowV), Scalar(BHighH, BHighS, BHighV), binBrightImg);
            //binBrightImg = channels[2];
            // Find all the contours in the thresholded image
            vector< vector<Point> > contours;
            imshow("pre_treatment",binBrightImg);
            findContours(binBrightImg.clone(), contours, RETR_LIST, CHAIN_APPROX_NONE);
            Mat drawingImg = Mat::zeros(frame.size(), CV_8UC3);

            for (size_t i = 0; i < contours.size(); i++){

                vector<Point> points;
                double area = contourArea(contours[i]);
                if (area < 20 || 1e3 < area) continue;
                drawContours(drawingImg, contours, static_cast<int>(i), Scalar(0), 2 );

                double high;
                points = contours[i];

                RotatedRect rrect = fitEllipse(points);//left_lightinfos

                cv::Point2f* vertices = new cv::Point2f[4];
                rrect.points(vertices);


                    for (int j = 0; j < 4; j++)
                    {
                        cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),4);//
                    }

                 //ellipse(binary,rrect,Scalar(0));
                high = rrect.size.height;

                for(size_t j = 1;j < contours.size();j++){

                     vector<Point> pointsA;
                     double area = contourArea(contours[j]);
                     if (area < 20 || 1e3 < area) continue;


                     double highA, distance;
                     double slop ;
                     pointsA = contours[j];

                     RotatedRect rrectA = fitEllipse(pointsA);//right_lightinfos

                     slop = abs(rrect.angle - rrectA.angle);
                     highA = rrectA.size.height;
                     distance  =sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                                     (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));


                     double max_height, min_height;
                     if(rrect.size.height > rrectA.size.height){
                         max_height = rrect.size.height;
                         min_height = rrectA.size.height;
                     }
                     else{
                         max_height = rrectA.size.height;
                         min_height = rrect.size.height;
                     }

                     double line_x = abs(rrect.center.x-rrectA.center.x);
                     double difference = max_height - min_height;
                     double aim =   distance/((highA+high)/2);
                     double difference3 = abs(rrect.size.width -rrectA.size.width);
                     double height = (rrect.size.height+rrectA.size.height)/200;
                     double slop_low = abs(rrect.angle + rrectA.angle)/2;


                     if((aim < 3.0 - height && aim > 2.0 - height     //小装甲板
                           && slop <= 5 && difference <=8 && difference3 <= 5
                          &&(slop_low <= 30 || slop_low >=150) && line_x >0.6*distance)
                          || (aim < 5.0-height && aim > 3.2 - height  //大装甲板
                              && slop <= 7 && difference <=15 && difference3 <= 8
                             &&(slop_low <= 30 || slop_low >=150) && line_x >0.7*distance)){
                         heights[hi] = (rrect.size.height+rrectA.size.height)/2;
                         R[hi] = rrect;
                         RA[hi] = rrectA;
                         hi++;
                     }
                 }
            }


            double max = 0;
            int mark;
            for(int i = 0;i < hi;i++){     //多个目标存在，打更近装甲板
                if(heights[i]  >= max){
                    max = heights[i];
                    mark = i;
                }
            }
            if(hi != 0){

                cv::circle(binary,Point((R[mark].center.x+RA[mark].center.x)/2,
                           (R[mark].center.y+RA[mark].center.y)/2),
                           15,cv::Scalar(0,0,255),4);

                Mat new_img = my_resize(frame, 28, 28);
                Mat data = new_img.reshape(1, 1);
                data.convertTo(data, CV_32FC1);
                Ptr<SVM> svm = StatModel::load<SVM>("/home/tyj/liantiao/src/hkvs/src/datasvm.xml");
                int result = (int)svm->predict(data);
                cout<<result<<endl;
                double center_x = (R[mark].center.x+RA[mark].center.x)/2;
                double center_y = (R[mark].center.y+RA[mark].center.y)/2;
                double height_equal = abs(R[mark].center.x-RA[mark].center.x)/2;
                double width_equal =  abs((R[mark].size.height+RA[mark].size.height)/4);

                Mat prediction = KF.predict();
                Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

                measurement.at<float>(0) = (float)center_x;
                measurement.at<float>(1) = (float)center_y;
                KF.correct(measurement);

                circle(binary, predict_pt, 3, Scalar(34, 255, 255), -1);

                center_x = (int)prediction.at<float>(0);
                center_y = (int)prediction.at<float>(1);
                cout<<"center_x="<<center_x<<"  "<<"center_y="<<center_y<<endl;

                vector<cv::Point3f> objectPoints;                //小装甲板空间坐标
                objectPoints.push_back(cv::Point3f(0, 0, 0));
                objectPoints.push_back(cv::Point3f(13.5, 0, 0));
                objectPoints.push_back(cv::Point3f(13.5, 5.4, 0));
                objectPoints.push_back(cv::Point3f(0, 5.4, 0));


                vector<cv::Point3f> objectPoints2;                //大装甲板空间坐标
                objectPoints2.push_back(cv::Point3f(0, 0, 0));
                objectPoints2.push_back(cv::Point3f(23, 0, 0));
                objectPoints2.push_back(cv::Point3f(23, 5.6, 0));
                objectPoints2.push_back(cv::Point3f(0, 5.6, 0));

                vector<cv::Point2f> imagePoints;                 //装甲板像素坐标

                float x1 =  center_x - height_equal, y1= center_y - width_equal;
                float x2 =  center_x +  height_equal, y2= center_y + width_equal;

                imagePoints.push_back(Point2f(x1, y1));
                imagePoints.push_back(Point2f(x2, y1));
                imagePoints.push_back(Point2f(x2, y2));
                imagePoints.push_back(Point2f(x1, y2));
                cout<<x1<<","<<y1<<endl;
                cout<<x2<<","<<y1<<endl;
                cout<<x2<<","<<y2<<endl;
                cout<<x1<<","<<y2<<endl;
        for (int j = 0; j < 4; j++)
        {
        if(j==0){squared.zs_x=x1;squared.zs_y=y1;}
        if(j==1){squared.ys_x=x2;squared.ys_y=y1;}
        if(j==2){squared.yx_x=x2;squared.yx_y=y2;}
        if(j==3){squared.zx_x=x1;squared.zx_y=y2;}
        }
	    cout<<"zuoshang"<<squared.zs_x<<".."<<squared.zs_y<<endl;
	    cout<<"youshang"<<squared.ys_x<<".."<<squared.ys_y<<endl;
	    cout<<"zuoxia"<<squared.zx_x<<".."<<squared.zx_y<<endl;
	    cout<<"youxia"<<squared.yx_x<<".."<<squared.yx_y<<endl;
 		squared.square_num=1;
		squarepub.publish(squared);           

                float whichone = abs(x1-x2)/abs(y1-y2);
                float loss = abs(whichone / 2.4 - 1)*100;
                cout << "误差:  "<< loss<< "%" << endl << endl; 



               double lessx  =  320 - center_x;   // .
               double lessy =   240 - center_y;
               cout << "坐标差"<<lessx << "  " << lessy << " "  <<endl;

            }

            imshow("okey",binary);
            waitKey(2);

            finish = clock();
            totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
            cout<<"Time whole"<<totaltime<<"秒"<<endl;
            hi = 0;
            
    }



    ros::spin();
}

