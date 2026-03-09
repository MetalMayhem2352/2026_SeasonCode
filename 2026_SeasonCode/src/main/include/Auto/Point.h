#pragma once

namespace Autonomous
{
    class Point
    {
        public:
            Point(double x, double y, double heading);
            ~Point();

            Point WithStartupDelay(double startWaitTime);
            Point WithEndDelay(double endWaitTime);
            Point WithReverseHeading();

            double x;
            double y;
            double heading;
            double startupDelay;
            double endDelay;
    };
}