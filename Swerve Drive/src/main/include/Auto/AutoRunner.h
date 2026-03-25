#pragma once

#include "Core/Timer.h"

#include "Auto/Point.h"


#include <initializer_list>
#include <queue>



namespace Autonomous
{
    class AutoRunner
    {
        private:

            std::queue<Point> points;
        public:
            AutoRunner();
            ~AutoRunner();

            void Update();

            void MakePaths(std::initializer_list<Point> points);

            std::queue<Point> ToQueue(std::initializer_list<Point> points);
    };
}