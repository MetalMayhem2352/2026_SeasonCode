#pragma once

#include "Modules/IntakeModule.h"
#include "Modules/TurretModule.h"
#include "Modules/ShooterModule.h"
#include "CustomSwerveDrive/SwerveDriveModule.h"
#include "CustomSwerveDrive/SwerveDriveOdometry.h"
#include "Core/Timer.h"

#include "Auto/Point.h"


#include <initializer_list>
#include <queue>



namespace Autonomous
{
    class AutoRunner
    {
        private:
            CustomSwerveDrive::SwerveDriveOdometry* swerveDriveOdometry;
            CustomSwerveDrive::SwerveDriveModule* swerveDriveModule;

            std::queue<Point> points;
        public:
            AutoRunner(CustomSwerveDrive::SwerveDriveModule* swerveDriveModule, CustomSwerveDrive::SwerveDriveOdometry* swerveDriveOdometry);
            ~AutoRunner();

            void Update();

            void MakePaths(std::initializer_list<Point> points);

            std::queue<Point> ToQueue(std::initializer_list<Point> points);
    };
}