#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "Map.h"
#include "System.h"

int main()
{
    std::cout << "AVP_SIM" << std::endl;
    AVP_SIM::System SIM;
    SIM.simulate();
}