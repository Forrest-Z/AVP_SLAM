#include "Map.h"

namespace AVP_SIM
{
    Map::Map()
    {
        std::string path2SemantiMap = "../Examples/semantic.bin";
        std::string path2MapParam = "../Examples/map_param.bin";
        std::string path2MapImage = "../Examples/map2.png";
        std::string path2MapImageWithSemantic = "../Examples/map.png";
        std::string path2MapFreeSpace = "../Examples/freeSpace.png";

        mFreeSpace = cv::imread(path2MapFreeSpace, cv::IMREAD_GRAYSCALE);
        mImage = cv::imread(path2MapImage, cv::IMREAD_COLOR);
        mImageWithAllSemantic = cv::imread(path2MapImageWithSemantic, cv::IMREAD_UNCHANGED);

        std::ifstream fin0(path2MapParam.c_str(), std::ifstream::in | std::ifstream::binary);
        if (fin0)
        {
            fin0.read((char*)&mMapParam, sizeof(mMapParam));
            fin0.close();
        }
        
        // extractEuclideanClusters(path2MapImageWithSemantic);
        readSemantic(path2SemantiMap);
        drawOnMap();

        _lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=100");
         if (!_lcm->good())
         {
             std::cout << "fail to initialize lcm" << std::endl;
         }
    }

    void Map::readSemantic(std::string &path2Semantic)
    {
        std::ifstream fin1(path2Semantic.c_str(), std::ifstream::in | std::ifstream::binary);

        int numSemantic;
        int numDottedLine;
        int numStraightArrow;
        int numArrowTurns;
        int numStraightTurningArrow;
        int numDoubleArrow;

        if (fin1)
        {
            fin1.read((char*)&numSemantic, sizeof(int));
            mvSemantic.resize(numSemantic);
            if (!mvSemantic.empty())
            {
                fin1.read((char*)&mvSemantic[0], numSemantic * sizeof(SemanticInformation));
            }
            // std::cout << "parking lot size: " << mvSemantic.size() << std::endl;


            fin1.read((char*)&numDottedLine, sizeof(int));
            mvDottedLine.resize(numDottedLine);
            if (!mvDottedLine.empty())
            {
                fin1.read((char*)&mvDottedLine[0], numDottedLine * sizeof(DottedLine));
            }

            // std::cout << "line size: " << mvDottedLine.size() << std::endl;

            fin1.read((char*)&numStraightArrow, sizeof(int));
            mvStraightArrow.resize(numStraightArrow);
            if (!mvStraightArrow.empty())
            {
                fin1.read((char*)&mvStraightArrow[0], numStraightArrow * sizeof(StraightArrow));
            }

            fin1.read((char*)&numArrowTurns, sizeof(int));
            mvArrowTurns.resize(numArrowTurns);
            if (!mvArrowTurns.empty())
            {
                fin1.read((char*)&mvArrowTurns[0], numArrowTurns * sizeof(ArrowTurns));
            }

            fin1.read((char*)&numStraightTurningArrow, sizeof(int));
            mvStraightTurningArrow.resize(numStraightTurningArrow);
            if (!mvStraightTurningArrow.empty())
            {
                fin1.read((char*)&mvStraightTurningArrow[0], numStraightTurningArrow * sizeof(StraightTurningArrow));
            }

            fin1.read((char*)&numDoubleArrow, sizeof(int));
            mvDoubleArrow.resize(numDoubleArrow);
            if (!mvDoubleArrow.empty())
            {
                fin1.read((char*)&mvDoubleArrow[0], numDoubleArrow * sizeof(DoubleArrow));
            }

            fin1.close();
        }
    }

    void Map::drawOnMap()
    {
        mImageWithSemantic = mImage.clone();

        // draw coordinate
        float x0 = 0.0;
        float y0 = 0.0;
        float x1 = 10.0;
        float y1 = 0.0;
        float x2 = 0.0;
        float y2 = 10.0;

        int col0 = (x0 - mMapParam.OriginX) / mMapParam.Res;
        int row0 = (y0 - mMapParam.OriginY) / mMapParam.Res;
        int col1 = (x1 - mMapParam.OriginX) / mMapParam.Res;
        int row1 = (y1 - mMapParam.OriginY) / mMapParam.Res;
        int col2 = (x2 - mMapParam.OriginX) / mMapParam.Res;
        int row2 = (y2 - mMapParam.OriginY) / mMapParam.Res;

        cv::arrowedLine(mImageWithSemantic, cv::Point2f(col0, row0), cv::Point2f(col1, row1), cv::Scalar(0,255,0),2);
        cv::arrowedLine(mImageWithSemantic, cv::Point2f(col0, row0), cv::Point2f(col2, row2), cv::Scalar(0,0,255),2);

        // draw semantics
        for (auto elem : mvSemantic)
        {
            float x1 = elem.cornerx1;
            float y1 = elem.cornery1;

            float x2 = elem.cornerx2;
            float y2 = elem.cornery2;

            int row1 = (y1 - mMapParam.OriginY) / mMapParam.Res;
            int col1 = (x1 - mMapParam.OriginX) / mMapParam.Res;

            int row2 = (y2 - mMapParam.OriginY) / mMapParam.Res;
            int col2 = (x2 - mMapParam.OriginX) / mMapParam.Res;
     
            cv::circle(mImageWithSemantic, cv::Point2f(col1, row1), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col2, row2), 2, cv::Scalar(0, 255, 0), -1);
        }

        for (auto elem : mvDottedLine)
        {
            float x0 = elem.x0;
            float y0 = elem.y0;
            int row0 = (y0 - mMapParam.OriginY) / mMapParam.Res;
            int col0 = (x0 - mMapParam.OriginX) / mMapParam.Res;
            cv::Point2f p1(x0, y0);

            float x1 = elem.x1;
            float y1 = elem.y1;
            int row1 = (y1 - mMapParam.OriginY) / mMapParam.Res;
            int col1 = (x1 - mMapParam.OriginX) / mMapParam.Res;
            cv::Point2f p2(x1, y1);

            cv::circle(mImageWithSemantic, cv::Point2f(col0, row0), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col1, row1), 2, cv::Scalar(0, 255, 0), -1);
        }

        for (auto elem : mvStraightArrow)
        {
            float x0 = elem.x0;
            float y0 = elem.y0;
            int row0 = (y0 - mMapParam.OriginY) / mMapParam.Res;
            int col0 = (x0 - mMapParam.OriginX) / mMapParam.Res;

            float x1 = elem.x1;
            float y1 = elem.y1;
            int row1 = (y1 - mMapParam.OriginY) / mMapParam.Res;
            int col1 = (x1 - mMapParam.OriginX) / mMapParam.Res;

            float x2 = elem.x2;
            float y2 = elem.y2;
            int row2 = (y2 - mMapParam.OriginY) / mMapParam.Res;
            int col2 = (x2 - mMapParam.OriginX) / mMapParam.Res;

            cv::circle(mImageWithSemantic, cv::Point2f(col0, row0), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col1, row1), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col2, row2), 2, cv::Scalar(0, 255, 0), -1);
        }

        for (auto elem : mvArrowTurns)
        {
            float x0 = elem.x0;
            float y0 = elem.y0;
            int row0 = (y0 - mMapParam.OriginY) / mMapParam.Res;
            int col0 = (x0 - mMapParam.OriginX) / mMapParam.Res;

            float x1 = elem.x1;
            float y1 = elem.y1;
            int row1 = (y1 - mMapParam.OriginY) / mMapParam.Res;
            int col1 = (x1 - mMapParam.OriginX) / mMapParam.Res;

            float x2 = elem.x2;
            float y2 = elem.y2;
            int row2 = (y2 - mMapParam.OriginY) / mMapParam.Res;
            int col2 = (x2 - mMapParam.OriginX) / mMapParam.Res;

            cv::circle(mImageWithSemantic, cv::Point2f(col0, row0), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col1, row1), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col2, row2), 2, cv::Scalar(0, 255, 0), -1);
        }

        for (auto elem : mvStraightTurningArrow)
        {
            float x0 = elem.x0;
            float y0 = elem.y0;
            int row0 = (y0 - mMapParam.OriginY) / mMapParam.Res;
            int col0 = (x0 - mMapParam.OriginX) / mMapParam.Res;

            float x1 = elem.x1;
            float y1 = elem.y1;
            int row1 = (y1 - mMapParam.OriginY) / mMapParam.Res;
            int col1 = (x1 - mMapParam.OriginX) / mMapParam.Res;

            float x2 = elem.x2;
            float y2 = elem.y2;
            int row2 = (y2 - mMapParam.OriginY) / mMapParam.Res;
            int col2 = (x2 - mMapParam.OriginX) / mMapParam.Res;

            cv::circle(mImageWithSemantic, cv::Point2f(col0, row0), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col1, row1), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col2, row2), 2, cv::Scalar(0, 255, 0), -1);
        }

        for (auto elem : mvDoubleArrow)
        {
            float x0 = elem.x0;
            float y0 = elem.y0;
            int row0 = (y0 - mMapParam.OriginY) / mMapParam.Res;
            int col0 = (x0 - mMapParam.OriginX) / mMapParam.Res;

            float x1 = elem.x1;
            float y1 = elem.y1;
            int row1 = (y1 - mMapParam.OriginY) / mMapParam.Res;
            int col1 = (x1 - mMapParam.OriginX) / mMapParam.Res;

            float x2 = elem.x2;
            float y2 = elem.y2;
            int row2 = (y2 - mMapParam.OriginY) / mMapParam.Res;
            int col2 = (x2 - mMapParam.OriginX) / mMapParam.Res;

            cv::circle(mImageWithSemantic, cv::Point2f(col0, row0), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col1, row1), 2, cv::Scalar(0, 255, 0), -1);
            cv::circle(mImageWithSemantic, cv::Point2f(col2, row2), 2, cv::Scalar(0, 255, 0), -1);
        }

        mImageGlobal = mImageWithSemantic.clone();
        mImageLocal = mImageWithSemantic.clone();
    }

    cv::Mat Map::bevSensing(const cv::Point3f &CarPose)
    {
        cv::Mat ret(800, 800, CV_32FC3, cv::Scalar(255,255,255));

        //draw Car
        float x0 = 20.0;
        float y0 = 20.0;
        float x1 = 1.5*cos(0.0) + x0;
        float y1 = 1.5*sin(0.0) + y0;

        int col_center = (x0) / mMapParam.Res;
        int row_center = (y0) / mMapParam.Res;
        int col_head = (x1) / mMapParam.Res;
        int row_head = (y1) / mMapParam.Res;

        cv::arrowedLine(ret, cv::Point2f(col_center, row_center), cv::Point2f(col_head, row_head), cv::Scalar(255,0,0));

        float xfl = 1.5*cos(0.0 - 0.5) + x0;
        float yfl = 1.5*sin(0.0 - 0.5) + y0;
        int col_fl = (xfl) / mMapParam.Res;
        int row_fl = (yfl) / mMapParam.Res;
        cv::circle(ret, cv::Point2f(col_fl, row_fl), 2, cv::Scalar(255,0,0), -1);

        float xfr = 1.5*cos(0.0 + 0.5) + x0;
        float yfr = 1.5*sin(0.0 + 0.5) + y0;
        int col_fr = (xfr) / mMapParam.Res;
        int row_fr = (yfr) / mMapParam.Res;
        cv::circle(ret, cv::Point2f(col_fr, row_fr), 2, cv::Scalar(255,0,0), -1);

        float xbr = -1.5*cos(0.0 + 0.5) + x0;
        float ybr = -1.5*sin(0.0 + 0.5) + y0;
        int col_br = (xbr) / mMapParam.Res;
        int row_br = (ybr) / mMapParam.Res;
        cv::circle(ret, cv::Point2f(col_br, row_br), 2, cv::Scalar(255,0,0), -1);

        float xbl = -1.5*cos(0.0 - 0.5) + x0;
        float ybl = -1.5*sin(0.0 - 0.5) + y0;
        int col_bl = (xbl) / mMapParam.Res;
        int row_bl = (ybl) / mMapParam.Res;
        cv::circle(ret, cv::Point2f(col_bl, row_bl), 2, cv::Scalar(255,0,0), -1);

        // search semantic can be seen
        // transfer point to car coordinate
        cv::Mat Twb = cv::Mat::eye(3,3,CV_32F);
        Twb.at<float>(0,0) = cos(CarPose.z); Twb.at<float>(0,1) = -1*sin(CarPose.z); Twb.at<float>(0,2) = CarPose.x;
        Twb.at<float>(1,0) = sin(CarPose.z); Twb.at<float>(1,1) = cos(CarPose.z); Twb.at<float>(1,2) = CarPose.y;
        // std::cout << "Twb: " << Twb << std::endl;

        int nSemantic = 0;
        for(auto elem : mvSemantic)
        {
            cv::Point2f temp1(elem.cornerx1, elem.cornery1);
            cv::Mat Pb1 = canBeSeen(Twb, temp1);
            if (!Pb1.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb1.at<float>(0) / mMapParam.Res;
                int row = Pb1.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,255,0), -1);
                nSemantic++;
            }

            cv::Point2f temp2(elem.cornerx2, elem.cornery2);
            cv::Mat Pb2 = canBeSeen(Twb, temp2);
            if (!Pb2.empty())
            {
                //std::cout << "Pb2: " << Pb2 << std::endl; 
                int col = Pb2.at<float>(0) / mMapParam.Res;
                int row = Pb2.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,255,0), -1);
                nSemantic++;
            }
        }

        for(auto elem : mvDottedLine)
        {
            cv::Point2f temp1(elem.x0, elem.y0);
            cv::Mat Pb1 = canBeSeen(Twb, temp1);
            if (!Pb1.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb1.at<float>(0) / mMapParam.Res;
                int row = Pb1.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }

            cv::Point2f temp2(elem.x1, elem.y1);
            cv::Mat Pb2 = canBeSeen(Twb, temp2);
            if (!Pb2.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb2.at<float>(0) / mMapParam.Res;
                int row = Pb2.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }
        }

        for(auto elem : mvStraightArrow)
        {
            cv::Point2f temp1(elem.x0, elem.y0);
            cv::Mat Pb1 = canBeSeen(Twb, temp1);
            if (!Pb1.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb1.at<float>(0) / mMapParam.Res;
                int row = Pb1.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }

            cv::Point2f temp2(elem.x1, elem.y1);
            cv::Mat Pb2 = canBeSeen(Twb, temp2);
            if (!Pb2.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb2.at<float>(0) / mMapParam.Res;
                int row = Pb2.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }

            cv::Point2f temp3(elem.x2, elem.y2);
            cv::Mat Pb3 = canBeSeen(Twb, temp3);
            if (!Pb3.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb3.at<float>(0) / mMapParam.Res;
                int row = Pb3.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }
        }

        for(auto elem : mvArrowTurns)
        {
            cv::Point2f temp1(elem.x0, elem.y0);
            cv::Mat Pb1 = canBeSeen(Twb, temp1);
            if (!Pb1.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb1.at<float>(0) / mMapParam.Res;
                int row = Pb1.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }

            cv::Point2f temp2(elem.x1, elem.y1);
            cv::Mat Pb2 = canBeSeen(Twb, temp2);
            if (!Pb2.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb2.at<float>(0) / mMapParam.Res;
                int row = Pb2.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }

            cv::Point2f temp3(elem.x2, elem.y2);
            cv::Mat Pb3 = canBeSeen(Twb, temp3);
            if (!Pb3.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb3.at<float>(0) / mMapParam.Res;
                int row = Pb3.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }
        }

        for(auto elem : mvStraightTurningArrow)
        {
            cv::Point2f temp1(elem.x0, elem.y0);
            cv::Mat Pb1 = canBeSeen(Twb, temp1);
            if (!Pb1.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb1.at<float>(0) / mMapParam.Res;
                int row = Pb1.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }

            cv::Point2f temp2(elem.x1, elem.y1);
            cv::Mat Pb2 = canBeSeen(Twb, temp2);
            if (!Pb2.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb2.at<float>(0) / mMapParam.Res;
                int row = Pb2.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }


            cv::Point2f temp3(elem.x2, elem.y2);
            cv::Mat Pb3 = canBeSeen(Twb, temp3);
            if (!Pb3.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb3.at<float>(0) / mMapParam.Res;
                int row = Pb3.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }
        }

        for(auto elem : mvDoubleArrow)
        {
            cv::Point2f temp1(elem.x0, elem.y0);
            cv::Mat Pb1 = canBeSeen(Twb, temp1);
            if (!Pb1.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb1.at<float>(0) / mMapParam.Res;
                int row = Pb1.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }

            cv::Point2f temp2(elem.x1, elem.y1);
            cv::Mat Pb2 = canBeSeen(Twb, temp2);
            if (!Pb2.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb2.at<float>(0) / mMapParam.Res;
                int row = Pb2.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }


            cv::Point2f temp3(elem.x2, elem.y2);
            cv::Mat Pb3 = canBeSeen(Twb, temp3);
            if (!Pb3.empty())
            {
                //std::cout << "Pb1: " << Pb1 << std::endl; 
                int col = Pb3.at<float>(0) / mMapParam.Res;
                int row = Pb3.at<float>(1) / mMapParam.Res;
                cv::circle(ret, cv::Point2f(col+400, row+400), 2, cv::Scalar(0,0,0), -1);
                nSemantic++;
            }
        }

        // std::cout << "nSemantic: " << nSemantic << std::endl;
        cv::imshow("semantic", ret);
        return ret;
    }

    cv::Mat Map::canBeSeen(cv::Mat &Twb, cv::Point2f point)
    {

        cv::Mat Pw = cv::Mat::ones(3,1,CV_32F);
        Pw.at<float>(0) = point.x;
        Pw.at<float>(1) = point.y;

        cv::Mat Pb = Twb.inv() * Pw;

        // // if can been see
        float dis = sqrt((Pb.at<float>(0))*(Pb.at<float>(0)) + (Pb.at<float>(1))*(Pb.at<float>(1)));


        if (dis < 8.0)
        {   
            return cv::Mat(Pb);
        }
        else
        {
            return cv::Mat();
        }
    }

    void Map::drawLocal(const cv::Point3f &CarPose, int width)
    {
        std::unique_lock<std::mutex> lock(mMutexLocal);
        
        int row_center = (CarPose.y - mMapParam.OriginY) / mMapParam.Res;
        int col_center = (CarPose.x - mMapParam.OriginX) / mMapParam.Res;

        int row1, row2, col1, col2;
        (row_center - width) <= 0 ? row1 = 0 : row1 = (row_center - width);
        (row_center + width) >= mImageWithSemantic.rows ? row2 = mImageWithSemantic.rows : row2 = (row_center + width);
        (col_center - width) <= 0 ? col1 = 0 : col1 = (col_center - width);
        (col_center + width) >= mImageWithSemantic.cols ? col2 = mImageWithSemantic.cols : col2 = (col_center + width);

        cv::Mat temp = mImageWithSemantic.clone();

        //draw car
        float x0 = CarPose.x;
        float y0 = CarPose.y;
        float x1 = 1.5*cos(CarPose.z) + x0;
        float y1 = 1.5*sin(CarPose.z) + y0;

        int col_head = (x1 - mMapParam.OriginX) / mMapParam.Res;
        int row_head = (y1 - mMapParam.OriginY) / mMapParam.Res;

        cv::circle(temp, cv::Point2f(col_center, row_center), 161, cv::Scalar(255,0,0));
        cv::arrowedLine(temp, cv::Point2f(col_center, row_center), cv::Point2f(col_head, row_head), cv::Scalar(255,0,0));

        float xfl = 1.5*cos(CarPose.z - 0.5) + x0;
        float yfl = 1.5*sin(CarPose.z - 0.5) + y0;
        int col_fl = (xfl - mMapParam.OriginX) / mMapParam.Res;
        int row_fl = (yfl - mMapParam.OriginY) / mMapParam.Res;
        cv::circle(temp, cv::Point2f(col_fl, row_fl), 2, cv::Scalar(255,0,0), -1);

        float xfr = 1.5*cos(CarPose.z + 0.5) + x0;
        float yfr = 1.5*sin(CarPose.z + 0.5) + y0;
        int col_fr = (xfr - mMapParam.OriginX) / mMapParam.Res;
        int row_fr = (yfr - mMapParam.OriginY) / mMapParam.Res;
        cv::circle(temp, cv::Point2f(col_fr, row_fr), 2, cv::Scalar(255,0,0), -1);

        float xbr = -1.5*cos(CarPose.z + 0.5) + x0;
        float ybr = -1.5*sin(CarPose.z + 0.5) + y0;
        int col_br = (xbr - mMapParam.OriginX) / mMapParam.Res;
        int row_br = (ybr - mMapParam.OriginY) / mMapParam.Res;
        cv::circle(temp, cv::Point2f(col_br, row_br), 2, cv::Scalar(255,0,0), -1);

        float xbl = -1.5*cos(CarPose.z - 0.5) + x0;
        float ybl = -1.5*sin(CarPose.z - 0.5) + y0;
        int col_bl = (xbl - mMapParam.OriginX) / mMapParam.Res;
        int row_bl = (ybl - mMapParam.OriginY) / mMapParam.Res;
        cv::circle(temp, cv::Point2f(col_bl, row_bl), 2, cv::Scalar(255,0,0), -1);

        cv::Mat LocalMap = cv::Mat(width * 2, width * 2, mImageWithSemantic.type());
        temp(cv::Rect(cv::Point2f(col1, row1), cv::Point2f(col2, row2))).copyTo(mImageLocal);
        cv::imshow("LocalMap", mImageLocal);
    }

    void Map::drawGlobal(const cv::Point3f &CarPose)
    {
        std::unique_lock<std::mutex> lock(mMutexGlobal);
        //draw on Map
        float _x = CarPose.x;
        float _y = CarPose.y;
        int col = (_x - mMapParam.OriginX) / mMapParam.Res;
        int row = (_y - mMapParam.OriginY) / mMapParam.Res;
      
        cv::circle(mImageGlobal, cv::Point2f(col, row), 2, cv::Scalar(0, 255, 0), -1);
    }
    
    void Map::update(const cv::Point3f &CarPose)
    {
        drawGlobal(CarPose);
        drawLocal(CarPose, 400);
        // bevSensing(CarPose);
        semanticSeg(CarPose);
    }

    // void Map::extractEuclideanClusters(std::string &path2Semantic)
    // {
    //     cv::Mat imSemantic = cv::imread(path2Semantic, cv::IMREAD_UNCHANGED);

    //     std::set<int> sStraightArrowPixelValues = {46,47,48};
    //     std::set<std::pair<int,int>> sStraightArrowPixelPositions;

    //     for (size_t i = 0; i < imSemantic.cols; i++)
    //     {
    //         for (size_t j = 0; j < imSemantic.rows; j++)
    //         {
    //             cv::Point temp(i,j);
    //             int value = imSemantic.at<unsigned char>(temp);
    //             if (sStraightArrowPixelValues.find(value) == sStraightArrowPixelValues.end())
    //                 continue;
    //             sStraightArrowPixelPositions.insert(std::make_pair(i,j));
    //         }
    //     }
         
    //     std::cout << "has: " << sStraightArrowPixelPositions.size() << " StraightArrowPixels" << std::endl;
    // }

    cv::Mat Map::semanticSeg(const cv::Point3f &CarPose)
    {
        static int64_t sequence = -1;
        int col_center = (CarPose.x - mMapParam.OriginX) / mMapParam.Res;
        int row_center = (CarPose.y - mMapParam.OriginY) / mMapParam.Res;
        float width = 200;
                
        // {
        //     //draw car
        // float x0 = CarPose.x;
        // float y0 = CarPose.y;
        // float x1 = 1.5*cos(CarPose.z) + x0;
        // float y1 = 1.5*sin(CarPose.z) + y0;

        // int col_head = (x1 - mMapParam.OriginX) / mMapParam.Res;
        // int row_head = (y1 - mMapParam.OriginY) / mMapParam.Res;

        // // cv::circle(temp, cv::Point2f(col_center, row_center), 161, cv::Scalar(255,0,0));
        // cv::arrowedLine(temp, cv::Point2f(col_center, row_center), cv::Point2f(col_head, row_head), cv::Scalar(255,0,0));

        // float xfl = 1.5*cos(CarPose.z - 0.5) + x0;
        // float yfl = 1.5*sin(CarPose.z - 0.5) + y0;
        // int col_fl = (xfl - mMapParam.OriginX) / mMapParam.Res;
        // int row_fl = (yfl - mMapParam.OriginY) / mMapParam.Res;
        // cv::circle(temp, cv::Point2f(col_fl, row_fl), 2, cv::Scalar(255,0,0), -1);

        // float xfr = 1.5*cos(CarPose.z + 0.5) + x0;
        // float yfr = 1.5*sin(CarPose.z + 0.5) + y0;
        // int col_fr = (xfr - mMapParam.OriginX) / mMapParam.Res;
        // int row_fr = (yfr - mMapParam.OriginY) / mMapParam.Res;
        // cv::circle(temp, cv::Point2f(col_fr, row_fr), 2, cv::Scalar(255,0,0), -1);

        // float xbr = -1.5*cos(CarPose.z + 0.5) + x0;
        // float ybr = -1.5*sin(CarPose.z + 0.5) + y0;
        // int col_br = (xbr - mMapParam.OriginX) / mMapParam.Res;
        // int row_br = (ybr - mMapParam.OriginY) / mMapParam.Res;
        // cv::circle(temp, cv::Point2f(col_br, row_br), 2, cv::Scalar(255,0,0), -1);

        // float xbl = -1.5*cos(CarPose.z - 0.5) + x0;
        // float ybl = -1.5*sin(CarPose.z - 0.5) + y0;
        // int col_bl = (xbl - mMapParam.OriginX) / mMapParam.Res;
        // int row_bl = (ybl - mMapParam.OriginY) / mMapParam.Res;
        // cv::circle(temp, cv::Point2f(col_bl, row_bl), 2, cv::Scalar(255,0,0), -1);
        // }

        int row1, row2, col1, col2;
        (row_center - width) <= 0 ? row1 = 0 : row1 = (row_center - width);
        (row_center + width) >= mImageWithSemantic.rows ? row2 = mImageWithSemantic.rows : row2 = (row_center + width);
        (col_center - width) <= 0 ? col1 = 0 : col1 = (col_center - width);
        (col_center + width) >= mImageWithSemantic.cols ? col2 = mImageWithSemantic.cols : col2 = (col_center + width);

        cv::Mat imLocal;
        mImageWithAllSemantic.rowRange(row1, row2).colRange(col1, col2).copyTo(imLocal);
        cv::Mat imSemanticSeg(imLocal.size(), CV_32FC3, cv::Scalar(0,0,0));
        // semantic segmentation

        std::set<int> sParkingLotPixelValues = {16};
        std::set<int> sDottedLinePixelValues = {64,65,66};
        std::set<int> sStraightArrowPixelValues = {46,47,48};
        std::set<int> sArrowTurnsPixelValues = {49,50,51,52,53,54};
        std::set<int> sStraightTurningArrowPixelValues = {55,56,57,58,59,60};
        std::set<int> sDoubleArrowValues = {61,62,63};
        
        std::vector<cv::Point> sParkingLotPixelUVs;
        std::vector<cv::Point> sDottedLinePixelUVs;
        std::vector<cv::Point> sStraightArrowPixelUVs;
        std::vector<cv::Point> sArrowTurnsPixelUVs;
        std::vector<cv::Point> sStraightTurningArrowPixelUVs;
        std::vector<cv::Point> sDoubleArrowUVs;


        // semanticMsg seg;

        poseMsg carpos;
        carpos.x = CarPose.x;
        carpos.y = CarPose.y;
        carpos.theta = CarPose.z;

        cv::Mat Tbw = cv::Mat::eye(2,2,CV_32F);
        Tbw.at<float>(0,0) = cos(CarPose.z); Tbw.at<float>(0,1) = -1*sin(CarPose.z);
        Tbw.at<float>(1,0) = sin(CarPose.z); Tbw.at<float>(1,1) = cos(CarPose.z);
 
        for (size_t i = 0; i < imLocal.cols; i++)
        {
            for (size_t j = 0; j < imLocal.rows; j++)
            {
                cv::Point temp(i,j);
                int value = imLocal.at<unsigned char>(temp);
                if(sParkingLotPixelValues.find(value) != sParkingLotPixelValues.end())
                {
                    sParkingLotPixelUVs.push_back(temp);
                    cv::circle(imSemanticSeg, temp, 1, cv::Scalar(0,255,0), -1);

                    // create lcm msg
                    cv::Mat Pw(2,1,CV_32F);
                    Pw.at<float>(0) = (static_cast<float>(i) - 200.f) * mMapParam.Res;
                    Pw.at<float>(1) = (static_cast<float>(j) - 200.f) * mMapParam.Res;

                    cv::Mat Pb = Tbw * Pw;
              
                    
                    pointMsg temp;
                    temp.sequence = sequence;
                    temp.carPose = carpos;
                    temp.type = temp.parkingLot;
                    temp.x = Pb.at<float>(0);
                    temp.y = Pb.at<float>(1);
                    temp.z = 0.0;

                    // seg.points.push_back(temp);
                    _lcm->publish("point", &temp);
                }
                else if (sDottedLinePixelValues.find(value) != sDottedLinePixelValues.end())
                {
                    sDottedLinePixelUVs.push_back(temp);
                    cv::circle(imSemanticSeg, temp, 1, cv::Scalar(0,255,0), -1);

                    // create lcm msg
                    cv::Mat Pw(2,1,CV_32F);
                    Pw.at<float>(0) = (i - 200) * mMapParam.Res;
                    Pw.at<float>(1) = (j - 200) * mMapParam.Res;

                    cv::Mat Pb = Tbw * Pw;
                    pointMsg temp;
                    temp.sequence = sequence;
                    temp.carPose = carpos;
                    temp.type = temp.dottedLine;
                    temp.x = Pb.at<float>(0);
                    temp.y = Pb.at<float>(1);
                    temp.z = 0.0;

                    // seg.points.push_back(temp);
                    _lcm->publish("point", &temp);

                }
                else if (sStraightArrowPixelValues.find(value) != sStraightArrowPixelValues.end())
                {
                    sStraightArrowPixelUVs.push_back(temp);
                    cv::circle(imSemanticSeg, temp, 1, cv::Scalar(0,255,0), -1);

                    // create lcm msg
                    cv::Mat Pw(2,1,CV_32F);
                    Pw.at<float>(0) = (i - 200) * mMapParam.Res;
                    Pw.at<float>(1) = (j - 200) * mMapParam.Res;

                    cv::Mat Pb = Tbw * Pw;
                    pointMsg temp;
                    temp.sequence = sequence;
                    temp.carPose = carpos;
                    temp.type = temp.straightArrow;
                    temp.x = Pb.at<float>(0);
                    temp.y = Pb.at<float>(1);
                    temp.z = 0.0;

                    // seg.points.push_back(temp);
                    _lcm->publish("point", &temp);

                }
                else if (sArrowTurnsPixelValues.find(value) != sArrowTurnsPixelValues.end())
                {
                    sArrowTurnsPixelUVs.push_back(temp);      
                    cv::circle(imSemanticSeg, temp, 1, cv::Scalar(0,255,0), -1);
                   
                    // create lcm msg
                    cv::Mat Pw(2,1,CV_32F);
                    Pw.at<float>(0) = (i - 200) * mMapParam.Res;
                    Pw.at<float>(1) = (j - 200) * mMapParam.Res;

                    cv::Mat Pb = Tbw * Pw;
                    pointMsg temp;
                    temp.sequence = sequence;
                    temp.carPose = carpos;
                    temp.type = temp.arrowTurns;
                    temp.x = Pb.at<float>(0);
                    temp.y = Pb.at<float>(1);
                    temp.z = 0.0;

                    // seg.points.push_back(temp);
                    _lcm->publish("point", &temp);


                }
                else if (sStraightTurningArrowPixelValues.find(value) != sStraightTurningArrowPixelValues.end())
                {
                    sStraightTurningArrowPixelUVs.push_back(temp);   
                    cv::circle(imSemanticSeg, temp, 1, cv::Scalar(0,255,0), -1);

                    // create lcm msg
                    cv::Mat Pw(2,1,CV_32F);
                    Pw.at<float>(0) = (i - 200) * mMapParam.Res;
                    Pw.at<float>(1) = (j - 200) * mMapParam.Res;

                    cv::Mat Pb = Tbw * Pw;
                    pointMsg temp;
                    temp.sequence = sequence;
                    temp.carPose = carpos;
                    temp.type = temp.straightTurningArrow;
                    temp.x = Pb.at<float>(0);
                    temp.y = Pb.at<float>(1);
                    temp.z = 0.0;

                    // seg.points.push_back(temp);
                    _lcm->publish("point", &temp);

                } 
                else if (sDoubleArrowValues.find(value) != sDoubleArrowValues.end())
                {
                    sDoubleArrowUVs.push_back(temp);
                    cv::circle(imSemanticSeg, temp, 1, cv::Scalar(0,255,0), -1);

                    // create lcm msg
                    cv::Mat Pw(2,1,CV_32F);
                    Pw.at<float>(0) = (i - 200) * mMapParam.Res;
                    Pw.at<float>(1) = (j - 200) * mMapParam.Res;

                    cv::Mat Pb = Tbw * Pw;
                    pointMsg temp;
                    temp.sequence = sequence;
                    temp.carPose = carpos;
                    temp.type = temp.doubleArrow;
                    temp.x = Pb.at<float>(0);
                    temp.y = Pb.at<float>(1);
                    temp.z = 0.0;

                    // seg.points.push_back(temp);
                    _lcm->publish("point", &temp);

                }
                else
                {
                    continue;              
                }
            }
        }

        sequence++;
        // std::cout << "n points: " << seg.points.size() << std::endl;
        cv::imshow("semantic", imSemanticSeg);
        return imLocal;
    }

}