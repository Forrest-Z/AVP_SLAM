#include "Localizer.h"

Localizer::Localizer()
{
    std::string path2MapParam = "../Examples/map_param.bin";
    std::ifstream fin0(path2MapParam.c_str(), std::ifstream::in | std::ifstream::binary);
    if (fin0)
    {
        fin0.read((char*)&mMapParam, sizeof(mMapParam));
        fin0.close();
    }

    mpMap.reset(new pcl::PointCloud<pcl::PointXYZ>());

    readMap();
}

void Localizer::readMap()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../Examples/map.pcd", *cloud) == -1) 
    {
        PCL_ERROR("Couldn't read file rabbit.pcd\n");
    }

    *mpMap = *cloud;
    std::cout << "map size: " << mpMap->size() << std::endl;
    
    // draw map
    mImMap = cv::imread("../Examples/map2.png", cv::IMREAD_COLOR);
    for (size_t i = 0; i < mpMap->size(); i++)
    {
        pcl::PointXYZ temp = mpMap->points[i];
        int col = (temp.x - mMapParam.OriginX) / 0.05;
        int row = (temp.y - mMapParam.OriginY) / 0.05;
        cv::circle(mImMap, cv::Point(col, row), 1, cv::Scalar(0,255,0), -1);
    }

    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map", mImMap);
    cv::waitKey(0);
}

cv::Vec3f Localizer::update(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, PoseData carPose)
{
    static bool first = true;
    static float last_x = 0;
    static float last_y = 0;
    static float last_z = 0;

    if (first)
    {
        last_x = carPose.x_noised;
        last_y = carPose.y_noised;
        last_z = carPose.z_noised;

        // icp between map and new cloud to get more accurate relative pose 

        cv::Mat Twb(3,3,CV_32FC1);
        Twb.at<float>(0,0) = cos(carPose.z_noised);Twb.at<float>(0,1) = -1*sin(carPose.z_noised);Twb.at<float>(0,2) = carPose.x_noised;
        Twb.at<float>(1,0) = sin(carPose.z_noised);Twb.at<float>(1,1) = cos(carPose.z_noised);Twb.at<float>(1,2) = carPose.y_noised;
        Twb.at<float>(2,0) = 0;Twb.at<float>(2,1) = 0;Twb.at<float>(2,2) = 1;
        float dx = Twb.at<float>(0,2);
        float dy = Twb.at<float>(1,2);
        float dz = atan2(Twb.at<float>(1,0), Twb.at<float>(0,0));
        cv::Point3f odom(dx, dy, dz);

        icp(mpMap, Cloud, odom);

        std::cout << "estimated: " << x_estimated << " " << y_estimated << " " << z_estimated << std::endl;

        first = false;
    }
    else
    {
        // icp between map and new cloud to get more accurate relative pose 
        cv::Mat Twb1(3,3,CV_32FC1);
        Twb1.at<float>(0,0) = cos(last_z);Twb1.at<float>(0,1) = -1*sin(last_z);Twb1.at<float>(0,2) = last_x;
        Twb1.at<float>(1,0) = sin(last_z);Twb1.at<float>(1,1) = cos(last_z);Twb1.at<float>(1,2) = last_y;
        Twb1.at<float>(2,0) = 0;Twb1.at<float>(2,1) = 0;Twb1.at<float>(2,2) = 1;

        cv::Mat Twb2(3,3,CV_32FC1);
        Twb2.at<float>(0,0) = cos(carPose.z_noised);Twb2.at<float>(0,1) = -1*sin(carPose.z_noised);Twb2.at<float>(0,2) = carPose.x_noised;
        Twb2.at<float>(1,0) = sin(carPose.z_noised);Twb2.at<float>(1,1) = cos(carPose.z_noised);Twb2.at<float>(1,2) = carPose.y_noised;
        Twb2.at<float>(2,0) = 0;Twb2.at<float>(2,1) = 0;Twb2.at<float>(2,2) = 1;

        cv::Mat Twb1_icp(3,3,CV_32FC1);
        Twb1_icp.at<float>(0,0) = cos(z_estimated);Twb1_icp.at<float>(0,1) = -1*sin(z_estimated);Twb1_icp.at<float>(0,2) = x_estimated;
        Twb1_icp.at<float>(1,0) = sin(z_estimated);Twb1_icp.at<float>(1,1) = cos(z_estimated);Twb1_icp.at<float>(1,2) = y_estimated;
        Twb1_icp.at<float>(2,0) = 0;Twb1_icp.at<float>(2,1) = 0;Twb1_icp.at<float>(2,2) = 1;

        cv::Mat Tb1b2 = Twb1.inv() * Twb2;
        cv::Mat Twb2_icp = Twb1_icp * Tb1b2;


        float dx = Twb2_icp.at<float>(0,2);
        float dy = Twb2_icp.at<float>(1,2);
        float dz = atan2(Twb2_icp.at<float>(1,0), Twb2_icp.at<float>(0,0));

        cv::Point3f odom(dx, dy, dz);
        // std::cout << "relative pose: " << odom << std::endl;

        icp(mpMap, Cloud, odom);

        last_x = carPose.x_noised;
        last_y = carPose.y_noised;
        last_z = carPose.z_noised;


        std::cout << "estimated: " << x_estimated << " " << y_estimated << " " << z_estimated << std::endl;
    }

    // draw estimated path and true path
    int col = (carPose.x_true - mMapParam.OriginX) / 0.05;
    int row = (carPose.y_true - mMapParam.OriginY) / 0.05;
    cv::circle(mImMap, cv::Point(col, row), 1, cv::Scalar(0,0,255), -1);

    col = (x_estimated - mMapParam.OriginX) / 0.05;
    row = (y_estimated - mMapParam.OriginY) / 0.05;
    cv::circle(mImMap, cv::Point(col, row), 1, cv::Scalar(0,255,0), -1);

    cv::Point3f carpose(x_estimated, y_estimated, z_estimated);
    showLocal(mImMap, carpose, 400, 5);
}

// calculate relative pose between two pointclouds
void Localizer::icp(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc, cv::Point3f odom)
{  
    // drawMatchCloud(CloudTarget, CloudSrc);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(CloudSrc);
    icp.setInputTarget(CloudTarget);
    icp.setMaximumIterations(100);
    icp.setMaxCorrespondenceDistance (0.05);

    // init guess 
    Eigen::Matrix4f initGuess;
    initGuess << cos(odom.z), -1*sin(odom.z), 0, odom.x,
                 sin(odom.z),    cos(odom.z), 0, odom.y,
                       0,          0, 1,  0,
                       0,          0, 0,  1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*temp, initGuess);
    bool icp_succeeded_or_not = icp.hasConverged();
    double match_score = icp.getFitnessScore();
    Eigen::Matrix4f Twb = icp.getFinalTransformation();   
    std::cout << "result: " << icp_succeeded_or_not << " score: " << match_score << std::endl;

    // float estimated_dx = Tb1b2(0,3);
    // float estimated_dy = Tb1b2(1,3);
    // float estimated_dz = atan2(Tb1b2(1,0),Tb1b2(0,0));
    // std::cout << "estimated odom: " << estimated_dx << " " << estimated_dy << " " << estimated_dz << std::endl;
 

    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
    // for (size_t i = 0; i < CloudSrc->size(); i++)
    // {
    //     Eigen::Vector4f Pb2;
    //     pcl::PointXYZ pt = CloudSrc->points[i];
    //     Pb2 << pt.x , pt.y, pt.z, 1;
    //     Eigen::Vector4f Pb1 = Tb1b2 * Pb2;
        
    //     pt.x = Pb1(0);
    //     pt.y = Pb1(1);
    //     pt.z = Pb1(2);

    //     temp2->points.push_back(pt);
    // }
    

    // update estimated pose
    // Eigen::Matrix4f Twb1;
    // Twb1 << cos(z_estimated), -1*sin(z_estimated), 0, x_estimated,
    //         sin(z_estimated),    cos(z_estimated), 0, y_estimated,
    //         0,          0, 1,  0,
    //         0,          0, 0,  1;     

    // Eigen::Matrix4f Twb2 = Twb1 * Tb1b2;
    x_estimated = Twb(0,3);
    y_estimated = Twb(1,3);
    z_estimated = atan2(Twb(1,0), Twb(0,0));
    
    // x_estimated += odom.x;
    // y_estimated += odom.y;
    // z_estimated += odom.z;

    // drawMatchCloud(CloudTarget, temp2);
}

void Localizer::showLocal(const cv::Mat &im, cv::Point3f &carpose, int width, int time)
{
    int row_center = (carpose.y - mMapParam.OriginY) / mMapParam.Res;
    int col_center = (carpose.x - mMapParam.OriginX) / mMapParam.Res;

    int row1, row2, col1, col2;
    (row_center - width) <= 0 ? row1 = 0 : row1 = (row_center - width);
    (row_center + width) >= mImMap.rows ? row2 = mImMap.rows : row2 = (row_center + width);
    (col_center - width) <= 0 ? col1 = 0 : col1 = (col_center - width);
    (col_center + width) >= mImMap.cols ? col2 = mImMap.cols : col2 = (col_center + width);
   
    cv::Mat LocalMap = cv::Mat(width * 2, width * 2, mImMap.type());
    mImMap(cv::Rect(cv::Point2f(col1, row1), cv::Point2f(col2, row2))).copyTo(LocalMap);
    cv::namedWindow("Local", cv::WINDOW_AUTOSIZE);
    cv::imshow("Local", LocalMap); 
    cv::waitKey(time);
}
