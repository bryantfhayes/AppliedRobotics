#ifndef __FISHINGROBOT_H
#define __FISHINGROBOT_H

#include "SSRobot.h"

class FishingRobot : public SSRobot{
    public:
        FishingRobot();
        ~FishingRobot();
        void grab();
        void toss();
};

#endif