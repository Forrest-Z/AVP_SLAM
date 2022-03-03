#include "System.h"

namespace AVP_SIM
{
    System::System()
    {
        // read Map
        mpMap = new Map();

        // execute motion controller
        mpController = new Motion(0,0,0);
        mpController->setMap(mpMap);

        cv::namedWindow("semantic", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("LocalMap", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("GlobalMap", cv::WINDOW_NORMAL);
    }

    void System::simulate()
    {
        std::thread thread_controller(&AVP_SIM::Motion::run, mpController);
        thread_controller.detach();
        while (1)
        {
            cv::Point3f currentCarPose = mpController->getCarPose();
            mpMap->update(currentCarPose);
            usleep(100000);
        }
    }
}