#pragma once

#include <iostream>
#include <unistd.h>
#include <mutex>
#include "opencv2/core.hpp"


// Motion类，仿真汽车的运动轨迹
// 根据键盘输入速度生成对应轨迹
class Motion
{
public:
    Motion(float x, float y, float theta);

    cv::Point3f getCarPose();
    cv::Point3f getCarPoseNoised();

    void update(const int& nKeyValue);

private:
    cv::RNG mRNG;

    float _x, _x_noised; //m
    float _y, _y_noised;
    float _theta, _theta_noised; //rad    
    std::mutex mMutexCarPose;

    float _linear_velocity = 0.0; // km/h
    float _angular_velocity = 0.0; // rad/s

    //config
    float Vx, Vr, nVx, nVr;
};
