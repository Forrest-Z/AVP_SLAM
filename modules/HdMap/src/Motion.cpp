#include "Motion.h"

Motion::Motion(float x, float y, float theta): _x(x), _y(y), _theta(theta), _x_noised(x), _y_noised(y), _theta_noised(theta)
{
    cv::FileStorage fs("../Examples/config.yaml", cv::FileStorage::READ);
    Vx = fs["Vx"];
    Vr = fs["Vr"];
    nVx = fs["nVx"];
    nVr = fs["nVr"];
}

void Motion::update(const int& nKeyValue)
{
    switch (nKeyValue)
    {
        case 81:
            _angular_velocity = -1*Vr; // turn left
            break;
        case 82:
            _linear_velocity = Vx; // forward
            break;
        case 83:
            _angular_velocity = Vr; // turn right
            break;
        case 84:
            _linear_velocity = -1*Vx; // backward
            break;
        case -1:                    // stop
            _linear_velocity = 0.0;
            _angular_velocity = 0.0;
            break;
        default:
            break;
    }

    float dt = 0.1; //s
    float ds = dt * _linear_velocity * 1000 / 3600;
    float dtheta = dt * _angular_velocity;

    float ds_noised = ds * (1 + mRNG.gaussian(nVx));
    float dtheta_noised = dtheta*(1 + mRNG.gaussian(nVr));

    // std::cout << "ds: " << ds << " ds_noised: " << ds_noised << " dtheta: " << dtheta << " dtheta_noised: " << dtheta_noised << std::endl;

    // update car pose 汽车中心位置
    _theta += dtheta;
    while (_theta < -1 * M_PI)
    {
        _theta += 2 * M_PI;
    }
    while (M_PI < _theta)
    {
        _theta -= 2 * M_PI;
    }
    
    _x += ds * cos(_theta - dtheta/2.0);
    _y += ds * sin(_theta - dtheta/2.0);

    _theta_noised += dtheta_noised;
    while (_theta_noised < -1 * M_PI)
    {
        _theta_noised += 2 * M_PI;
    }
    while (M_PI < _theta_noised)
    {
        _theta_noised -= 2 * M_PI;
    }
    
    _x_noised += ds_noised * cos(_theta_noised - dtheta_noised/2.0);
    _y_noised += ds_noised * sin(_theta_noised - dtheta_noised/2.0);

    // std::cout << "x: " << _x << " y: " << _y << " theta: " << _theta << std::endl;
    // std::cout << "x_noised: " << _x_noised << " y_noised: " << _y_noised << " theta_noised: " << _theta_noised << std::endl;
}

cv::Point3f Motion::getCarPose()
{
    std::unique_lock<std::mutex> lock(mMutexCarPose);
    return cv::Point3f(_x, _y, _theta);
}

cv::Point3f Motion::getCarPoseNoised()
{
    std::unique_lock<std::mutex> lock(mMutexCarPose);
    return cv::Point3f(_x_noised, _y_noised, _theta_noised);
}
