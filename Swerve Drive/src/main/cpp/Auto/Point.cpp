#include "Auto/Point.h"

namespace Autonomous
{
    Point::Point(double x, double y, double heading)
    {
        startupDelay = 0;
        endDelay = 0;
    }

    Point::~Point()
    {

    }

    Point Point::WithStartupDelay(double startWaitTime)
    {
        startupDelay = startWaitTime;
        return *this; 
    }

    Point Point::WithEndDelay(double endWaitTime)
    {
        endWaitTime = endWaitTime;
        return *this; 
    }
    
    Point Point::WithReverseHeading()
    {
        double tempHeading = heading - 180;
        tempHeading *= -1;
        heading = tempHeading + 180;
        return *this; 
    }
}