#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <vector>
 
using namespace cv;
using namespace std;
// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
Mat getStructuringElement(int shape, Size ksize, Point anchor = Point(-1,-1));
void PressEnterToExit(void)
{
    int c;
    while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}


//
cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
{
	using std::swap;

	float& width = rec.size.width;
	float& height = rec.size.height;
	float& angle = rec.angle;

	if(mode == 0)
	{
		if(width < height)
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	while(angle >= 90.0) angle -= 180.0;
	while(angle < -90.0) angle += 180.0;

	if(mode == 1)
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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "hkvs_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    ros::Rate loop_rate(5);
    int nRet = MV_OK;
    int ready = 0;
    int end = 0;

    void* handle = NULL;
	unsigned char * pData = NULL;        
    unsigned char *pDataForRGB = NULL;
    unsigned char *pDataForSaveImage = NULL;
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    Mat src;
    sensor_msgs::ImagePtr msg;

   while (1)
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Intput camera index: ");
        unsigned int nIndex = 0;
        //scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }
		
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }
		
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            break;
        }

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            break;
        }

        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            break;
        }
        unsigned int nDataSize = stParam.nCurValue;

        nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            printf("Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n", 
                stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);


            // 处理图像
            // image processing
            printf("input 0 to do nothing, 1 to convert RGB, 2 to save as BMP\n");
            int nInput = 1;
            //scanf("%d", &nInput);
            
                    pDataForRGB = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
                    if (NULL == pDataForRGB)
                    {
                        break;
                    }
                    // 像素格式转换
                    // convert pixel format 
                    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
                    // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
                    // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
                    // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format, 
                    // destination pixel format, output data buffer, provided output buffer size
                    stConvertParam.nWidth = stImageInfo.nWidth;
                    stConvertParam.nHeight = stImageInfo.nHeight;
                    stConvertParam.pSrcData = pData;
                    stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
                    stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
                    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
                    stConvertParam.pDstBuffer = pDataForRGB;
                    stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight *  4 + 2048;
                    nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
                    if (MV_OK != nRet)
                    {
                        printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                        break;
                    }

                    FILE* fp = fopen("AfterConvert_RGB.raw", "wb");
                    if (NULL == fp)
                    {
                        printf("fopen failed\n");
                        break;
                    }
                    fwrite(pDataForRGB, 1, stConvertParam.nDstLen, fp);

                    cv::Mat src(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB);
                    //cv::imshow("a",src);
                    Mat image;
                    Mat binBrightImg;
                    
                    //Mat dstImage = Mat::zeros(src.rows, src.cols, CV_8UC3);
                    cvtColor(src,image,COLOR_BGR2GRAY,1);
                    cv::threshold(image, binBrightImg, 210, 255, cv::THRESH_BINARY);
                    cv::Mat element = cv::getStructuringElement(2, cv::Size(3, 3));
                    dilate(binBrightImg, binBrightImg, element);
                    cv::imshow("b",binBrightImg);

                    vector<vector<Point> > contours;
 
	                //使用canny检测出边缘
	                //Mat edge_image;
	                //Canny(binBrightImg,edge_image,30,70);
	                //cv::imshow("canny边缘",edge_image);
                    findContours(binBrightImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	                Mat cimage = Mat::zeros(binBrightImg.size(), CV_8UC3);
 
	                for(size_t i = 0; i < contours.size(); i++)
	               {
		            //拟合的点至少为6
		            size_t count = contours[i].size();
		            if( count < 6 )
			        continue;
 
		            //椭圆拟合
		            RotatedRect box = fitEllipse(contours[i]);
 
		            //如果长宽比大于30，则排除，不做拟合
		            if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
			        continue;
 
		            //画出追踪出的轮廓
		            drawContours(cimage, contours, (int)i, Scalar::all(255), 1, 8);
		
		            //画出拟合的椭圆
		            ellipse(cimage, box, Scalar(0,0,255), 1, CV_AA);
	               }
                   if(!cimage.empty())
	               cv::imshow("results", cimage);

                    /*vector<vector<Point> > lightContours;
                    vector<Vec4i> hierarchy;
                    findContours(binBrightImg.clone(), lightContours,hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
                    std::vector<Rect> boundRect(lightContours.size());
                    int index = 0;
	                for (int i =0;i<lightContours.size();i++)
                    {
                       // 获取最小外接矩形
                       boundRect[i] = boundingRect(lightContours[i]);

                       // 在原图像上绘制最小外接矩形
                       rectangle(src, boundRect[i], Scalar(0, 255, 0));
                    }
                    if(!src.empty())
                    imshow("result", src);*/
                    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
                  if(msg==0)
                  {
                   printf("open error\n");
                  }
                     
                     
                     
                       cv::waitKey(5);
                   
                    
                    


                    fclose(fp);
                    printf("convert succeed\n");
             
              
        }

        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            break;
        }
    } ;
     
    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }
    if (pData)
    {
        free(pData);	
        pData = NULL;
    }
    if (pDataForRGB)
    {
        free(pDataForRGB);
        pDataForRGB = NULL;
    }
    if (pDataForSaveImage)
    {
        free(pDataForSaveImage);
        pDataForSaveImage = NULL;
    }

    while (nh.ok())
    {
        
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    PressEnterToExit();
    printf("exit\n");
    cv::waitKey(0);
    return 0;
}
