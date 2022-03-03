#pragma once

#include <iostream>
#include <unistd.h>
#include <mutex>
#include "Map.h"

namespace AVP_SIM
{
    // Motion类，仿真汽车的运动轨迹
    // 根据键盘输入速度生成对应轨迹
    class Motion
    {
    public:
        Motion(float x, float y, float theta);
        void run();

        void setMap(Map* pMap);

        cv::Point3f getCarPose();

    private:
        void setTwist(const int& nKeyValue);
        void update();

    private:
        float _x; //m
        float _y;
        float _theta; //rad    
        std::mutex mMutexCarPose;

        float _linear_velocity = 0.0; // km/h
        float _angular_velocity = 0.0; // rad/s

        Map* mpMap;
        cv::Mat mImageTrajectory;
        cv::Mat mImageLocal;
        std::mutex mMutexImageLocal;
    };
}