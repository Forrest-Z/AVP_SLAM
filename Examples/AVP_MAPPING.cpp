#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "pointMsg.hpp"
#include <string>

int loadLCM(std::string path2lcm)
{
 // Open the log file.
    lcm::LogFile log(path2lcm, "r");
    if (!log.good()) {
        perror("LogFile");
        fprintf(stderr, "couldn't open log file %s\n", path2lcm);
        return 1;
    }

    while (1) {
        // Read a log event.
        const lcm::LogEvent *event = log.readNextEvent();
        if (!event)
            break;

        // Only process messages on the EXAMPLE channel.
        if (event->channel != "point")
            continue;

        // Try to decode the message.
        pointMsg msg;
        if (msg.decode(event->data, 0, event->datalen) != event->datalen)
            continue;

        // Decode success!  Print out the message contents.
        std::cout << "point:\n" << std::endl;
        std::cout << "sequence: " << msg.sequence << " x: " << msg.x << " y: " << msg.y << std::endl;
        std::cout << "car pose: " << msg.carPose.x << " " << msg.carPose.y << " " << msg.carPose.theta << std::endl;

    }

    // Log file is closed automatically when the log variable goes out of
    // scope.

    printf("read done\n");
    return 0;
}

int main(int argc, char **argv)
{
    loadLCM("../data/lcmlog-2022-03-04.00");  
}