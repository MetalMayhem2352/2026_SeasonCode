#include "Auto/AutoRunner.h"

namespace Auto
{
    AutoRunner::AutoRunner(CustomSwerveDrive::SwerveDriveModule* swerveDriveModule, Modules::IntakeModule* intakeModule, 
        Modules::ShooterModule* shooterModule, Modules::BasketModule* basketModule, Turret_Tracking* turretModule)
    {
        swerveDriveOdometry = swerveDriveModule->CreateSwerveDriveOdometery();
        this->swerveDriveModule = swerveDriveModule;
        this->intakeModule = intakeModule;
        this->shooterModule = shooterModule;
        this->basketModule = basketModule;
        this->turretModule = turretModule;
        
        autoTimer = new Core::Timer();
    }
    AutoRunner::~AutoRunner()
    {
        delete(autoTimer);
        delete(swerveDriveOdometry);
    }

    void AutoRunner::Update()
    {
        autoTimer->Update();
    }

}