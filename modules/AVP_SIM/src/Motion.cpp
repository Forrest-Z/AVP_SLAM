#include "Motion.h"

namespace AVP_SIM
{
    Motion::Motion(float x, float y, float theta): _x(x), _y(y), _theta(theta){}

    void Motion::setMap(Map* pMap)
    {
        mpMap = pMap;
    }

    void Motion::run()
    {
        while (1)
        {
            mImageTrajectory = mpMap->mImageGlobal.clone();
            cv::imshow("GlobalMap", mImageTrajectory);
            int key = cv::waitKey(1);
            // std::cout << key << std::endl;
            setTwist(key);
            update();
        }        
    }

    void Motion::setTwist(const int& nKeyValue)
    {
        switch (nKeyValue)
        {
            case 81:
                _angular_velocity = -0.5; // turn left
                break;
            case 82:
                _linear_velocity = 7.0; // forward
                break;
            case 83:
                _angular_velocity = 0.5; // turn right
                break;
            case 84:
                _linear_velocity = -7.0; // backward
                break;
            case -1:                    // stop
                _linear_velocity = 0.0;
                _angular_velocity = 0.0;
                break;
            default:
                break;
        }
    }

    void Motion::update()
    {
        float dt = 0.1; //s
        float ds = dt * _linear_velocity * 1000 / 3600;
        float dtheta = dt * _angular_velocity;

        // update car pose 汽车中心位置
        _theta += dtheta;
        _x += ds * cos(_theta - dtheta/2.0);
        _y += ds * sin(_theta - dtheta/2.0);

        //std::cout << "x: " << _x << " y: " << _y << " theta: " << _theta << std::endl;
    }

    cv::Point3f Motion::getCarPose()
    {
        std::unique_lock<std::mutex> lock(mMutexCarPose);
        return cv::Point3f(_x, _y, _theta);
    }

}