#include "hdMap.h"

hdMap* hdMap::_instance = 0;

hdMap::hdMap()
{
    std::cout << "initialize hd map" << std::endl;
    std::string path2MapParam = "../Examples/map_param.bin";
    std::string path2MapImageWithSemanticInfo = "../Examples/map.png";
    std::string path2MapImage = "../Examples/map2.png";
    mImMap = cv::imread(path2MapImage, cv::IMREAD_COLOR);

    
    std::ifstream fin0(path2MapParam.c_str(), std::ifstream::in | std::ifstream::binary);
    if (fin0)
    {
        fin0.read((char*)&mMapParam, sizeof(mMapParam));
        fin0.close();
    }

    cv::Mat tempImSemantic = cv::imread(path2MapImageWithSemanticInfo, cv::IMREAD_UNCHANGED);
    // readSemantic(tempImSemantic);
    readSemanticPCL(tempImSemantic);

    cv::FileStorage fs("../Examples/config.yaml", cv::FileStorage::READ);
    nPixel = fs["nPixel"];
}

hdMap* hdMap::Instance()
{
    if (_instance == 0)
    {
        _instance = new hdMap();
    }
    return _instance;
}
/**
void hdMap::readSemantic(const cv::Mat & ImSemantic)
{
    std::set<int> sParkingLotPixelValues = {16};
    std::set<int> sDottedLinePixelValues = {64,65,66};
    std::set<int> sStraightArrowPixelValues = {46,47,48};
    std::set<int> sArrowTurnsPixelValues = {49,50,51,52,53,54};
    std::set<int> sStraightTurningArrowPixelValues = {55,56,57,58,59,60};
    std::set<int> sDoubleArrowValues = {61,62,63};

    for (size_t col = 0; col < ImSemantic.cols; col++)
    {
        for (size_t row = 0; row < ImSemantic.rows; row++)
        {
            int value = ImSemantic.at<unsigned char>(col, row);

            if(sParkingLotPixelValues.find(value) != sParkingLotPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                pointType pt(x, y, 0.0, parkingLot);
                mvSemantics.push_back(pt);
            }
            else if (sDottedLinePixelValues.find(value) != sDottedLinePixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                pointType pt(x, y, 0.0, dottedLine);
                mvSemantics.push_back(pt);
            }
            else if (sStraightArrowPixelValues.find(value) != sStraightArrowPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                pointType pt(x, y, 0.0, straightArrow);
                mvSemantics.push_back(pt);
            }
            else if (sArrowTurnsPixelValues.find(value) != sArrowTurnsPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                pointType pt(x, y, 0.0, arrowTurns);
                mvSemantics.push_back(pt);
            }
            else if (sStraightTurningArrowPixelValues.find(value) != sStraightTurningArrowPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                pointType pt(x, y, 0.0, straightTurningArrow);
                mvSemantics.push_back(pt);
            } 
            else if (sDoubleArrowValues.find(value) != sDoubleArrowValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                pointType pt(x, y, 0.0, doubleArrow);
                mvSemantics.push_back(pt);
            }
            else
            {
                continue;              
            }
        }
    }
    
    std::cout << "msSemantic size: " << mvSemantics.size() << std::endl;
}
**/
void hdMap::readSemanticPCL(const cv::Mat & ImSemantic)
{
    // std::set<int> sParkingLotPixelValues = {16};
    // std::set<int> sDottedLinePixelValues = {64,65,66};
    // std::set<int> sStraightArrowPixelValues = {46,47,48};
    // std::set<int> sArrowTurnsPixelValues = {49,50,51,52,53,54};
    // std::set<int> sStraightTurningArrowPixelValues = {55,56,57,58,59,60};
    // std::set<int> sDoubleArrowValues = {61,62,63};

    // only corners
    std::set<int> sParkingLotPixelValues = {};
    std::set<int> sDottedLinePixelValues = {64};
    std::set<int> sStraightArrowPixelValues = {46};
    std::set<int> sArrowTurnsPixelValues = {49,52};
    std::set<int> sStraightTurningArrowPixelValues = {55,58};
    std::set<int> sDoubleArrowValues = {61};

    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    for (size_t col = 0; col < ImSemantic.cols; col++)
    {
        for (size_t row = 0; row < ImSemantic.rows; row++)
        {
            int value = ImSemantic.at<unsigned char>(row, col);

            if(sParkingLotPixelValues.find(value) != sParkingLotPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                // pcl::PointXYZ pt(x, y, parkingLot);
                pcl::PointXYZ pt(x, y, 0.0);
                pointcloud.push_back(pt);
            }
            else if (sDottedLinePixelValues.find(value) != sDottedLinePixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                // pcl::PointXYZ pt(x, y, dottedLine);
                pcl::PointXYZ pt(x, y, 0.0);
                pointcloud.push_back(pt);
            }
            else if (sStraightArrowPixelValues.find(value) != sStraightArrowPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                // pcl::PointXYZ pt(x, y, straightArrow);
                pcl::PointXYZ pt(x, y, 0.0);
                pointcloud.push_back(pt);
            }
            else if (sArrowTurnsPixelValues.find(value) != sArrowTurnsPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                // pcl::PointXYZ pt(x, y, arrowTurns);
                pcl::PointXYZ pt(x, y, 0.0);
                pointcloud.push_back(pt);
            }
            else if (sStraightTurningArrowPixelValues.find(value) != sStraightTurningArrowPixelValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                // pcl::PointXYZ pt(x, y, straightTurningArrow);
                pcl::PointXYZ pt(x, y, 0.0);
                pointcloud.push_back(pt);
            } 
            else if (sDoubleArrowValues.find(value) != sDoubleArrowValues.end())
            {
                float x = col * mMapParam.Res + mMapParam.OriginX;
                float y = row * mMapParam.Res + mMapParam.OriginY;
                // pcl::PointXYZ pt(x, y,  doubleArrow);
                pcl::PointXYZ pt(x, y,  0.0);
                pointcloud.push_back(pt);
            }
            else
            {
                continue;              
            }
        }
    }
    
    std::cout << "pointcloud size: " << pointcloud.size() << std::endl;
    mpSemanticsPCL = pointcloud.makeShared();
    // pcl::visualization::PCLVisualizer viewer("semantics");
    // viewer.addPointCloud(mpSemanticsPCL);
    // viewer.spin();
    mpKDtree.setInputCloud(mpSemanticsPCL);

    //check if true
    // for (size_t i = 0; i < mpSemanticsPCL->size(); i++)
    // {
    //     pcl::PointXYZ temp = mpSemanticsPCL->points[i];
    //     int col = (temp.x - mMapParam.OriginX) / mMapParam.Res;
    //     int row = (temp.y - mMapParam.OriginY) / mMapParam.Res;
    //     cv::circle(mImMap, cv::Point(col, row), 2, cv::Scalar(0,255,0), -1);
    // }
    // cv::namedWindow("map", cv::WINDOW_NORMAL);
    // cv::imshow("map", mImMap);
    // cv::waitKey(0);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr hdMap::observe(cv::Point3f carPose)
{
    std::vector<int> indices;     
    std::vector<float> distances; 
    pcl::PointXYZ point(carPose.x, carPose.y, carPose.z);

    double radius = 8.0;
    int size = mpKDtree.radiusSearch(point, radius, indices, distances);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ret(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*mpSemanticsPCL, indices, *ret);
    // std::cout << "search point : " << size << " " << ret->size() << std::endl;

    //transform to car coordinate and add gaussian noise~(0, 1pixels)
    cv::RNG rng;
    cv::Mat Twb(3,3,CV_32FC1);
    Twb.at<float>(0,0) = cos(carPose.z);Twb.at<float>(0,1) = -1*sin(carPose.z);Twb.at<float>(0,2) = carPose.x;
    Twb.at<float>(1,0) = sin(carPose.z);Twb.at<float>(1,1) = cos(carPose.z);Twb.at<float>(1,2) = carPose.y;
    Twb.at<float>(2,0) = 0;Twb.at<float>(2,1) = 0;Twb.at<float>(2,2) = 1;
    cv::Mat Tbw = Twb.inv();
    for (size_t i = 0; i < ret->size(); i++)
    {
        // transform to car coordinate
        cv::Mat Pw(3,1,CV_32F);
        Pw.at<float>(0) = ret->points[i].x;
        Pw.at<float>(1) = ret->points[i].y;
        Pw.at<float>(2) = 1;
        // std::cout << "x: " << Pw.at<float>(0) << " y: " << Pw.at<float>(1) << std::endl;
        // std::cout << "Tbw: " << Tbw << std::endl;
        cv::Mat Pb = Tbw * Pw;
        // std::cout << "x: " << Pb.at<float>(0) << " y: " << Pb.at<float>(1) << std::endl;

        // add noise
        double noiseX = rng.gaussian(nPixel);
        double noiseY = rng.gaussian(nPixel);
        Pb.at<float>(0) += noiseX*mMapParam.Res;
        Pb.at<float>(1) += noiseY*mMapParam.Res;

        ret->points[i].x = Pb.at<float>(0);
        ret->points[i].y = Pb.at<float>(1);
    }
    
    return ret;
}
