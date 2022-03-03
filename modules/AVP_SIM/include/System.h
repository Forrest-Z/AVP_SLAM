#pragma once

#include "Map.h"
#include "Motion.h"
#include "thread"
#include <unistd.h>

namespace AVP_SIM
{
    class System
    {
    public:
        System();
        void simulate();

    private:
        Map* mpMap;
        Motion* mpController;
    };
}