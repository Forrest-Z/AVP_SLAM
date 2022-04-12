#include "LocalMapper.h"

unsigned int LocalMapper::index = 0;

LocalMapper::LocalMapper():bFirst(true), bFinish(false)
{
    nId = index++;
    mpLocalMap.reset(new pcl::PointCloud<pcl::PointXYZ>());
    mpLastFrame.reset(new pcl::PointCloud<pcl::PointXYZ>());
    max_points = mpLocalMap->size();

    std::string path2MapImage = "../Examples/map2.png";
    mImMap = cv::imread(path2MapImage, cv::IMREAD_COLOR);

    std::string path2MapParam = "../Examples/map_param.bin";
    std::ifstream fin0(path2MapParam.c_str(), std::ifstream::in | std::ifstream::binary);
    if (fin0)
    {
        fin0.read((char*)&mMapParam, sizeof(mMapParam));
        fin0.close();
    }
}

// create a local map
void LocalMapper::update(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, cv::Point3f carPose)
{
    
    if (bFirst)
    {
        *mpLocalMap = *Cloud;
        *mpLastFrame = *Cloud;
        pose = carPose;
        lastCarPose = carPose;
        bFirst = false;
    }
    else
    {
        drawMatchCloud(mpLastFrame, Cloud);
        cv::Mat Twb1(3,3,CV_32FC1);
        Twb1.at<float>(0,0) = cos(pose.z);Twb1.at<float>(0,1) = -1*sin(pose.z);Twb1.at<float>(0,2) = pose.x;
        Twb1.at<float>(1,0) = sin(pose.z);Twb1.at<float>(1,1) = cos(pose.z);Twb1.at<float>(1,2) = pose.y;
        Twb1.at<float>(2,0) = 0;Twb1.at<float>(2,1) = 0;Twb1.at<float>(2,2) = 1;

        cv::Mat Twb2(3,3,CV_32FC1);
        Twb2.at<float>(0,0) = cos(carPose.z);Twb2.at<float>(0,1) = -1*sin(carPose.z);Twb2.at<float>(0,2) = carPose.x;
        Twb2.at<float>(1,0) = sin(carPose.z);Twb2.at<float>(1,1) = cos(carPose.z);Twb2.at<float>(1,2) = carPose.y;
        Twb2.at<float>(2,0) = 0;Twb2.at<float>(2,1) = 0;Twb2.at<float>(2,2) = 1;

        cv::Mat Tb1b2 = Twb1.inv() * Twb2;

        for (size_t i = 0; i < Cloud->size(); i++)
        {
            cv::Mat Pb2(3,1,CV_32F);
            Pb2.at<float>(0) = Cloud->points[i].x;
            Pb2.at<float>(1) = Cloud->points[i].y;
            Pb2.at<float>(2) = 1;

            cv::Mat Pb1 = Tb1b2 * Pb2;

            Cloud->points[i].x = Pb1.at<float>(0);
            Cloud->points[i].y = Pb1.at<float>(1);
        }

        drawMatchCloud(mpLastFrame, Cloud);
        *mpLocalMap += *Cloud;

        pcl::PCLPointCloud2::Ptr cloudIn(new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2::Ptr cloudOut(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*mpLocalMap, *cloudIn);

        pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
        vox.setInputCloud (cloudIn);
        vox.setLeafSize(0.05f,0.05f,0.05f);
        vox.filter (*cloudOut);

        pcl::fromPCLPointCloud2(*cloudOut, *mpLocalMap);
    }


    showLocalMap();

    if (sqrt(pow((carPose.x-pose.x),2)+pow((carPose.y-pose.y),2))>5)
    {
        bFirst = true;
        mpLocalMap->clear();
    }
}

void LocalMapper::showLocalMap()
{
    std::cout << "local map size: " << mpLocalMap->size() << std::endl;
    cv::Mat tempMap = mImMap.clone();

    cv::Mat Twb(3,3,CV_32FC1);
    Twb.at<float>(0,0) = cos(pose.z);Twb.at<float>(0,1) = -1*sin(pose.z);Twb.at<float>(0,2) = pose.x;
    Twb.at<float>(1,0) = sin(pose.z);Twb.at<float>(1,1) = cos(pose.z);Twb.at<float>(1,2) = pose.y;
    Twb.at<float>(2,0) = 0;Twb.at<float>(2,1) = 0;Twb.at<float>(2,2) = 1;
    for (size_t i = 0; i < mpLocalMap->size(); i++)
    {
        pcl::PointXYZ temp = mpLocalMap->points[i];
        cv::Mat Pb(3,1,CV_32F);
        Pb.at<float>(0) = temp.x;
        Pb.at<float>(1) = temp.y;
        Pb.at<float>(2) = 1;

        cv::Mat Pw = Twb * Pb;

        int col = (Pw.at<float>(0) - mMapParam.OriginX) / mMapParam.Res;
        int row = (Pw.at<float>(1) - mMapParam.OriginY) / mMapParam.Res;
        cv::circle(tempMap, cv::Point(col, row), 1, cv::Scalar(0,255,0), -1);
    }
    
    //show local
    int row_center = (pose.y - mMapParam.OriginY) / mMapParam.Res;
    int col_center = (pose.x - mMapParam.OriginX) / mMapParam.Res;

    int row1, row2, col1, col2;
    (row_center - 400) <= 0 ? row1 = 0 : row1 = (row_center - 400);
    (row_center + 400) >= mImMap.rows ? row2 = mImMap.rows : row2 = (row_center + 400);
    (col_center - 400) <= 0 ? col1 = 0 : col1 = (col_center - 400);
    (col_center + 400) >= mImMap.cols ? col2 = mImMap.cols : col2 = (col_center + 400);

    cv::Mat LocalMap = cv::Mat(400 * 2, 400 * 2, mImMap.type());
    tempMap(cv::Rect(cv::Point2f(col1, row1), cv::Point2f(col2, row2))).copyTo(LocalMap);

    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map", LocalMap);
    cv::waitKey(5);
}

void LocalMapper::drawMatchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc)
{
    cv::Mat Imtemp(2000,2000,CV_32FC3,cv::Scalar(255,255,255));
    for (size_t i = 0; i < CloudTarget->size(); i++)
    {
        pcl::PointXYZ temp = CloudTarget->points[i];
        int col = temp.x / 0.01;
        int row = temp.y / 0.01;
        cv::circle(Imtemp, cv::Point(col+1000, row+1000), 1, cv::Scalar(0,255,0), -1);
    }

    for (size_t i = 0; i < CloudSrc->size(); i++)
    {
        pcl::PointXYZ temp = CloudSrc->points[i];
        int col = temp.x / 0.01;
        int row = temp.y / 0.01;
        cv::circle(Imtemp, cv::Point(col+1000, row+1000), 1, cv::Scalar(255,0,0), -1);    
    }
    cv::namedWindow("localMapper", cv::WINDOW_NORMAL);
    cv::imshow("localMapper", Imtemp);
    cv::waitKey(5);
}