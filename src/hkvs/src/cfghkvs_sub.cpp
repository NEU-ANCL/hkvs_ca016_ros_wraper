#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <hkvs/square.h>
#include <iostream>
using namespace cv;
#include <dynamic_reconfigure/server.h>
#include <hkvs/hkvsConfig.h>
#include<stdio.h>
#include<math.h>
#include<vector>
using namespace std;

cv_bridge::CvImagePtr cv_ptr;
int frame_number=0;
cv::Mat img;

 //灯条长度的差//
 double heightDiff;
//灯条宽度的差//
 double widthDiff;
double angleDiff;
//计算左右灯条长度的均值//
double meanheight;
//灯条y的比率//
double yDiffRatio;
//灯条x的比率//
double xDiffRatio;
//计算左右灯条中心距离//
double dis;
//灯条的距离与长度的比值（也就是嫌疑装甲板长和宽的比值）//
double ratio;
float heightDiff_ratio;

class ImageConverter
{
private:
    ros::NodeHandle nh_;
    //用于将msg信息转换为openCV中的Mat数据
    image_transport::ImageTransport it_;
    //订阅摄像头发布的信息
    image_transport::Subscriber image_sub_;
    hkvs::square squared;

    ros::Publisher squarepub = nh_.advertise<hkvs::square>("point2d", 1000);
public:
    ImageConverter()
    : it_(nh_)
    {
        //设置订阅摄像机

        image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    }

    ~ImageConverter(){
    }

    //收到摄像机后的回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
        try{
            //将收到的消息使用cv_bridge转移到全局变量图像指针cv_ptr中，其成员变量image就是Mat型的图片
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }

        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //处理图片信息
        img=cv_ptr->image;
        image_process(img); //得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数   
    }
    /*
       这是图象处理的主要函数，一般会把图像处理的主要程序写在这个函数中。这里的例子只是一个彩色图象到灰度图象的转化
    */

   enum
{
	WIDTH_GREATER_THAN_HEIGHT,
	ANGLE_TO_UP
};
cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
{
	using std::swap;

	float& width = rec.size.width;
	float& height = rec.size.height;
	float& angle = rec.angle;

	if(mode == WIDTH_GREATER_THAN_HEIGHT)
	{
		if(width < height)
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	while(angle >= 90.0) angle -= 180.0;
	while(angle < -90.0) angle += 180.0;

	if(mode == ANGLE_TO_UP)
	{
		if(angle >= 45.0)
		{
			swap(width, height);
			angle -= 90.0;
		}
		else if(angle < -45.0)
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	return rec;
}

class LightDescriptor
{
public:
	LightDescriptor() {};
	LightDescriptor(const cv::RotatedRect& light)
	{
		width = light.size.width;
		length = light.size.height;
		center = light.center;
		angle = light.angle;
		area = light.size.area();
	}
	const LightDescriptor& operator =(const LightDescriptor& ld)
	{
		this->width = ld.width;
		this->length = ld.length;
		this->center = ld.center;
		this->angle = ld.angle;
		this->area = ld.area;
		return *this;
	}

	/*
	*	@Brief: return the light as a cv::RotatedRect object
	*/
	cv::RotatedRect rec() const
	{
		return cv::RotatedRect(center, cv::Size2f(width, length), angle);
	}

public:
	float width;
	float length;
	cv::Point2f center;
	float angle;
	float area;
};

class ArmorRect
{
public:
    RotatedRect armors;
};

void drawall(vector<RotatedRect> rec,Mat img)
{
    for (int i = 0; i < rec.size(); i++)
    {
        Point2f p[4];
        rec[i].points(p);
        line(img, p[0], p[1], Scalar(0, 0, 255), 1, 8, 0);
        line(img, p[1], p[2], Scalar(0, 0, 255), 1, 8, 0);
        line(img, p[2], p[3], Scalar(0, 0, 255), 1, 8, 0);
        line(img, p[3], p[0], Scalar(0, 0, 255), 1, 8, 0);
        cout<<"top_left: "<<p[0]<<endl;
        cout<<"top_right: "<<p[1]<<endl;
        cout<<"bottom_left: "<<p[2]<<endl;
        cout<<"bottom_right: "<<p[3]<<endl;
	for (int j = 0; j < 4; j++){
        if(j==0){squared.zs_x=p[j].x;squared.zs_y=p[j].y;}
        if(j==1){squared.ys_x=p[j].x;squared.ys_y=p[j].y;}
        if(j==2){squared.yx_x=p[j].x;squared.yx_y=p[j].y;}
        if(j==3){squared.zx_x=p[j].x;squared.zx_y=p[j].y;}}
                        
    }
 squared.square_num=1;
	squarepub.publish(squared); 
}

    void image_process(cv::Mat img) 
    {
        cv::Mat img_out;
        Mat binBrightImg;
        Mat image;
        /*
        *    pre-treatment
        */
        cvtColor(img,image,COLOR_BGR2GRAY,1);
        //二值化 膨胀
        cv::threshold(image, binBrightImg, 210, 255, cv::THRESH_BINARY);
        cv::Mat element = cv::getStructuringElement(2, cv::Size(3, 3));
        dilate(binBrightImg, binBrightImg, element);
        cv::imshow("pre-treatment",binBrightImg);
        /*
        *    find and filter light bars
        */
        vector<vector<Point> >lightcontours;
        vector<RotatedRect> lightInfos;
        vector<RotatedRect>rect;
        //寻找轮廓，放入findContours中
        findContours(binBrightImg.clone(), lightcontours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        lightInfos.clear();
        for(const auto& contour : lightcontours)//对每个轮廓处理
        {
            float lightContourArea = contourArea(contour);//得到轮廓面积
            if(contour.size() <= 5 ||
            lightContourArea < 10) continue;
            //拟合成相应矩形
            RotatedRect lightRec = fitEllipse(contour);
            RotatedRect minAreaRec = minAreaRect(contour);
            adjustRec(lightRec, ANGLE_TO_UP);
            //筛选出需要的灯条
            if(lightRec.size.width / lightRec.size.height > 1 ) continue;
            //拓展长宽
            lightRec.size.width *= 1.1;
            lightRec.size.height *= 1.1;
            //获取外接矩形框
            Rect boundRect = lightRec.boundingRect();
            /*缺颜色识别*/
            //把灯条放入lightInfos中
            lightInfos.push_back(lightRec);
        }
        /*
        *	find and filter light bar pairs
            #### 约束:
            >1. 两个灯条近似平行
            >2. 两个灯条的高度大小近似相似
            >3. 中心点 y 坐标差距不大
            >4. 装甲板区域的长宽比在一定范围内  
        */
        vector<RotatedRect> armors;
        vector<ArmorRect> armorRects;
        ArmorRect armorRect;

        armors.clear();
        armorRects.clear();
        if (lightInfos.size()<=1)
        {
            cout << "There's no light contours in quality." << endl;
        }
        sort(lightInfos.begin(), lightInfos.end(), [](const RotatedRect& ld1, const RotatedRect& ld2)
        {
            return ld1.center.x < ld2.center.x;
        });

    for (int i = 0; i < lightInfos.size(); i++)
        {
            for (int j = i + 1; j < lightInfos.size(); j++)
                {
                    const RotatedRect& left = lightInfos[i];
                    const RotatedRect& right = lightInfos[j];
                    double yDiff = abs(left.center.y - right.center.y);
                    double xDiff = abs(left.center.x - right.center.x);



/*
                    //灯条长度的差//
                    double heightDiff = abs(left.size.height - right.size.height);
                    //灯条宽度的差//
                    double widthDiff = abs(left.size.width - right.size.width);
                    double angleDiff = abs(left.angle - right.angle);
                    //计算左右灯条长度的均值//
                    double meanheight = (left.size.height + right.size.height)/2;
                    //灯条y的比率//
                    double yDiffRatio = yDiff / meanheight;
                    //灯条x的比率//
                    double xDiffRatio = xDiff / meanheight;
                    //计算左右灯条中心距离//
                    double dis= sqrt((left.center.x - right.center.x)*(left.center.x - right.center.x) + (left.center.y - right.center.y)*(left.center.y - right.center.y));
                    //灯条的距离与长度的比值（也就是嫌疑装甲板长和宽的比值）//
                    double ratio = dis / meanheight;
                    float heightDiff_ratio = heightDiff / max(left.size.height, right.size.height);
*/
                    if (angleDiff > 10 || xDiffRatio < 0.5 || yDiffRatio>0.7||ratio>3||ratio<1)
                    continue;
                    armorRect.armors.center.x = (left.center.x + right.center.x) / 2;
                    armorRect.armors.center.y = (left.center.y + right.center.y) / 2;
                    armorRect.armors.angle= (left.angle + right.angle) / 2;
                    if (180 - angleDiff < 3)
                    armorRect.armors.angle += 90;
                    armorRect.armors.size.height= (left.size.height + right.size.height) / 2;
                    armorRect.armors.size.width = sqrt((left.center.x - right.center.x)*(left.center.x - right.center.x) + (left.center.y - right.center.y)*(left.center.y - right.center.y));
                    double nL = armorRect.armors.size.height;
                    double nW = armorRect.armors.size.width;
                    if (nL < nW)
                        {
                            armorRect.armors.size.height = nL;
                            armorRect.armors.size.width = nW;
                        }
                    else
                        {
                            armorRect.armors.size.height = nW;
                            armorRect.armors.size.width = nL;
                        }
                    //debug
                    cout<<"armors_height"<<nW<<endl;
                    cout<<"armors_width"<<nL<<endl;
                    armorRects.emplace_back(armorRect);
                    armors.push_back(armorRect.armors);
                }
        }
    if (armorRects.empty())
        cout << "There is no armor in quality!" << endl;
    
    drawall(armors, img);
    cv::imshow("", img);

                                



       cv::waitKey(5);
    }
};

void callback(hkvs::hkvsConfig &config, uint32_t level) {
                    double heightDiff=config.heightDiff;
                    double widthDiff=config.widthDiff;
                    double angleDiff=config.angleDiff;
                    double meanheight=config.meanheight;
                    double yDiffRatio=config.yDiffRatio;
                    double xDiffRatio=config.xDiffRatio;
                    double dis=config.dis;
                    double ratio=config.ratio;
                    float heightDiff_ratio=config.heightDiff_ratio;
			ROS_INFO("233333333333333333333333333333");
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    dynamic_reconfigure::Server<hkvs::hkvsConfig> server;
    dynamic_reconfigure::Server<hkvs::hkvsConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    ros::Rate loop_rate(10);
    server.setCallback(f);
    while(ros::ok())
    {
	
    ImageConverter ic;
    //循环等待
    ros::spin();
    loop_rate.sleep();
    }

}
