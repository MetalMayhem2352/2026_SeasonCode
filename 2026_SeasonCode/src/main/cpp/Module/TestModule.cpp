#include "Modules/TestModule.h"

namespace Modules
{
    TestModule::TestModule()
    {
        flywheelMotor1 = new ctre::phoenix6::hardware::TalonFX(0, "Defualt Name");
        flywheelMotor2 = new ctre::phoenix6::hardware::TalonFX(0, "Defualt Name");

        flywheelMotor2->GetConfigurator().Apply(Constants::Turret::shooterMotor2);
    }

    TestModule::~TestModule()
    {
        delete(flywheelMotor1);
        delete(flywheelMotor2);
    }

    void TestModule::Shoot()
    {
        flywheelMotor1->Set(0.3);
        flywheelMotor2->Set(0.3);
    }

    void TestModule::Stop()
    {
        flywheelMotor1->Set(0);
        flywheelMotor2->Set(0);
    }
}