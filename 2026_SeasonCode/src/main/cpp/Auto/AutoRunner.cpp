#include "Auto/AutoRunner.h"

namespace Autonomous
{
    AutoRunner::AutoRunner(CustomSwerveDrive::SwerveDriveModule* swerveDriveModule, CustomSwerveDrive::SwerveDriveOdometry* swerveDriveOdometry)
    {
        this->swerveDriveOdometry = swerveDriveOdometry;
        this->swerveDriveModule = swerveDriveModule;
    }
    AutoRunner::~AutoRunner()
    {
        delete(swerveDriveOdometry);
        delete(swerveDriveOdometry);
    }

    void AutoRunner::Update()
    {
        CustomDriveBase::RobotPosition currentPosition = swerveDriveOdometry->GetRobotPosition();
        double x = points.front().x - currentPosition.x;
        double z = points.front().y - currentPosition.z;

        x = x > 1 ? 1: x;
        z = z > 1 ? 1: z;

        swerveDriveModule->MoveFieldCentric2(x / 3, z / 3, points.front().heading);

        swerveDriveModule->Update();
        
        if (x < 0.25 && z > 0.25)
        {
            points.pop();
        }
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