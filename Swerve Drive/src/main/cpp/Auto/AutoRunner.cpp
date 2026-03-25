#include "Auto/AutoRunner.h"

namespace Autonomous
{
    AutoRunner::AutoRunner()
    {
    }
    AutoRunner::~AutoRunner()
    {
    }

    void AutoRunner::Update()
    {
        
    }

    
    void AutoRunner::MakePaths(std::initializer_list<Point> points)
    {
        this->points = ToQueue(points);
    }

    std::queue<Point> AutoRunner::ToQueue(std::initializer_list<Point> points) {
        std::queue<Point> queue;
        for (const auto& point : points) {
            queue.push(point);
        }
        return queue;
    }


}