#include "Modules/TestModule.h"

namespace Modules
{
    TestModule::TestModule()
    {
        flywheelMotor1 = new ctre::phoenix6::hardware::TalonFX(15, "Default Name");

        flywheelMotor1->GetConfigurator().Apply(Constants::Turret::shooterMotor1);
    }

    TestModule::~TestModule()
    {
        delete(flywheelMotor1);
    }

    void TestModule::Shoot()
    {
        flywheelMotor1->Set(1);
    }

    void TestModule::Stop()
    {
        flywheelMotor1->Set(0);
    }
}