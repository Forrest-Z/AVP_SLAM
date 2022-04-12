#include "Mapping.h"

Mapping::Mapping()
{
    std::string path2MapImage = "../Examples/map2.png";
    mImMap = cv::imread(path2MapImage, cv::IMREAD_COLOR);

    std::string path2MapParam = "../Examples/map_param.bin";
    std::ifstream fin0(path2MapParam.c_str(), std::ifstream::in | std::ifstream::binary);
    if (fin0)
    {
        fin0.read((char*)&mMapParam, sizeof(mMapParam));
        fin0.close();
    }

    mpMap.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

void Mapping::update(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, PoseData carPose)
{
    static bool first = true;
    static float last_x = 0;
    static float last_y = 0;
    static float last_z = 0;

    if (first)
    {
        mLastCloud = *Cloud;
        
        x_estimated = carPose.x_noised;
        y_estimated = carPose.y_noised;
        z_estimated = carPose.z_noised;

        last_x = carPose.x_noised;
        last_y = carPose.y_noised;
        last_z = carPose.z_noised;

        first = false;
        cv::waitKey(0);
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

        cv::Mat Tb1b2 = Twb1.inv() * Twb2;
        float dx = Tb1b2.at<float>(0,2);
        float dy = Tb1b2.at<float>(1,2);
        float dz = atan2(Tb1b2.at<float>(1,0), Tb1b2.at<float>(0,0));

        cv::Point3f odom(dx, dy, dz);
        std::cout << "relative pose: " << odom << std::endl;

        icp(mLastCloud.makeShared(), Cloud, odom);
        // icp_map(mpMap, Cloud, cv::Point3f(x_estimated, y_estimated, z_estimated));
        last_x = carPose.x_noised;
        last_y = carPose.y_noised;
        last_z = carPose.z_noised;

        mLastCloud = *Cloud;

        std::cout << "estimated: " << x_estimated << " " << y_estimated << " " << z_estimated << std::endl;
        std::cout << "carPose: " << carPose.x_noised << " " << carPose.y_noised << " " << carPose.z_noised << std::endl;

        // transform to world coordinate
        // cv::Point3f _carPose(carPose.x_noised, carPose.y_noised, carPose.z_noised);
        cv::Mat Twb(3,3,CV_32FC1);
        Twb.at<float>(0,0) = cos(z_estimated);Twb.at<float>(0,1) = -1*sin(z_estimated);Twb.at<float>(0,2) = x_estimated;
        Twb.at<float>(1,0) = sin(z_estimated);Twb.at<float>(1,1) = cos(z_estimated);Twb.at<float>(1,2) = y_estimated;
        Twb.at<float>(2,0) = 0;Twb.at<float>(2,1) = 0;Twb.at<float>(2,2) = 1;

        for (size_t i = 0; i < Cloud->size(); i++)
        {
            cv::Mat Pb(3,1,CV_32F);
            Pb.at<float>(0) = Cloud->points[i].x;
            Pb.at<float>(1) = Cloud->points[i].y;
            Pb.at<float>(2) = 1;

            cv::Mat Pw = Twb * Pb;

            Cloud->points[i].x = Pw.at<float>(0);
            Cloud->points[i].y = Pw.at<float>(1);
        }
    }
    
    // merge and downsample
    *mpMap += *Cloud;
    pcl::PCLPointCloud2::Ptr cloudIn(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloudOut(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*mpMap, *cloudIn);

    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
	vox.setInputCloud (cloudIn);
	vox.setLeafSize(0.2f,0.2f,0.2f);
    vox.filter (*cloudOut);

    pcl::fromPCLPointCloud2(*cloudOut, *mpMap);
    std::cout << "map size: " << mpMap->size() << std::endl;

    cv::Mat tempMap = mImMap.clone();

    for (size_t i = 0; i < mpMap->size(); i++)
    {
        pcl::PointXYZ temp = mpMap->points[i];
        int col = (temp.x - mMapParam.OriginX) / mMapParam.Res;
        int row = (temp.y - mMapParam.OriginY) / mMapParam.Res;
        cv::circle(tempMap, cv::Point(col, row), 1, cv::Scalar(0,255,0), -1);
    }
    
    //draw path
    int col = (carPose.x_true - mMapParam.OriginX) / mMapParam.Res;
    int row = (carPose.y_true - mMapParam.OriginY) / mMapParam.Res;
    cv::circle(mImMap, cv::Point(col,row), 1, cv::Scalar(0,255,0), -1);

    col = (x_estimated - mMapParam.OriginX) / mMapParam.Res;
    row = (y_estimated - mMapParam.OriginY) / mMapParam.Res;
    cv::circle(mImMap, cv::Point(col,row), 1, cv::Scalar(0,0,255), -1);

    col = (carPose.x_noised - mMapParam.OriginX) / mMapParam.Res;
    row = (carPose.y_noised - mMapParam.OriginY) / mMapParam.Res;
    cv::circle(mImMap, cv::Point(col,row), 1, cv::Scalar(255,0,0), -1);

    //show local
    int row_center = (y_estimated - mMapParam.OriginY) / mMapParam.Res;
    int col_center = (x_estimated - mMapParam.OriginX) / mMapParam.Res;

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

// calculate relative pose between two pointclouds
void Mapping::icp(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc, cv::Point3f odom)
{  
    drawMatchCloud(CloudTarget, CloudSrc);

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
    Eigen::Matrix4f Tb1b2 = icp.getFinalTransformation();   
    std::cout << "result: " << icp_succeeded_or_not << " score: " << match_score << std::endl;

    float estimated_dx = Tb1b2(0,3);
    float estimated_dy = Tb1b2(1,3);
    float estimated_dz = atan2(Tb1b2(1,0),Tb1b2(0,0));
    std::cout << "estimated odom: " << estimated_dx << " " << estimated_dy << " " << estimated_dz << std::endl;
 

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < CloudSrc->size(); i++)
    {
        Eigen::Vector4f Pb2;
        pcl::PointXYZ pt = CloudSrc->points[i];
        Pb2 << pt.x , pt.y, pt.z, 1;
        Eigen::Vector4f Pb1 = Tb1b2 * Pb2;
        
        pt.x = Pb1(0);
        pt.y = Pb1(1);
        pt.z = Pb1(2);

        temp2->points.push_back(pt);
    }
    

    // update estimated pose
    Eigen::Matrix4f Twb1;
    Twb1 << cos(z_estimated), -1*sin(z_estimated), 0, x_estimated,
            sin(z_estimated),    cos(z_estimated), 0, y_estimated,
            0,          0, 1,  0,
            0,          0, 0,  1;     

    Eigen::Matrix4f Twb2 = Twb1 * Tb1b2;
    x_estimated = Twb2(0,3);
    y_estimated = Twb2(1,3);
    z_estimated = atan2(Twb2(1,0), Twb2(0,0));
    
    // x_estimated += odom.x;
    // y_estimated += odom.y;
    // z_estimated += odom.z;

    drawMatchCloud(CloudTarget, temp2);
}

void Mapping::icp_map(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc, cv::Point3f odom)
{  
    drawMatchCloud(CloudTarget, CloudSrc);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(CloudSrc);
    icp.setInputTarget(CloudTarget);
    icp.setMaximumIterations(50);
    icp.setMaxCorrespondenceDistance (0.2);

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
    Eigen::Matrix4f Tb1b2 = icp.getFinalTransformation();   
    // std::cout << "result: " << icp_succeeded_or_not << " score: " << match_score << std::endl;
    std::cout << "odom: " << odom << std::endl;
    std::cout << "T12: " << Tb1b2 << std::endl;

    // update estimated pose


    Eigen::Matrix4f Twb2 = Tb1b2;
    x_estimated = Twb2(0,3);
    y_estimated = Twb2(1,3);
    z_estimated = atan2(Twb2(1,0), Twb2(0,0));
    
    // x_estimated += odom.x;
    // y_estimated += odom.y;
    // z_estimated += odom.z;

    drawMatchCloud(CloudTarget, temp);
}

void Mapping::drawMatchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc)
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

void Mapping::saveMap(const std::string filename)
{
    pcl::PCDWriter writer;
    writer.write(filename, *mpMap);
}
