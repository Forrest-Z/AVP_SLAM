#pragma once
#include "opencv2/core.hpp"
#include <vector>

class semantic
{
public:
    enum TYPE{parkingLot, dottedLine, straightArrow, arrowTurns, straightTurningArrow, doubleArrow}; 

private:
    TYPE type;

    std::vector<cv::Point> mvUVs; // 所有的像素位置
    std::vector<cv::Point> mvCorners; // 角点的像素位置
    std::vector<cv::Point> mvEdges; // 边线的像素位置
};
