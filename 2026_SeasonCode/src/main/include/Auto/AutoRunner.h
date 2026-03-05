#pragma once

#include "Modules/BasketModule.h"
#include "Modules/IntakeModule.h"
#include "Modules/TurretModule.h"
#include "Modules/ShooterModule.h"
#include "CustomSwerveDrive/SwerveDriveModule.h"
#include "CustomSwerveDrive/SwerveDriveOdometry.h"
#include "Core/Timer.h"


namespace Auto
{
    class AutoRunner
    {
        private:
            CustomSwerveDrive::SwerveDriveOdometry* swerveDriveOdometry;
            CustomSwerveDrive::SwerveDriveModule* swerveDriveModule;
            Modules::IntakeModule* intakeModule;
            Modules::ShooterModule* shooterModule;
            Modules::BasketModule* basketModule;
            Turret_Tracking* turretModule;

            Core::Timer* autoTimer;

        public:
            AutoRunner(CustomSwerveDrive::SwerveDriveModule* swerveDriveModule, Modules::IntakeModule* intakeModule, 
                Modules::ShooterModule* shooterModule, Modules::BasketModule* basketModule, Turret_Tracking* turretModule);
            ~AutoRunner();

            void Update();
    };
}