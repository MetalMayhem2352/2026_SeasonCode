#include "Modules/TestModule.h"

#include "Constants.h"

namespace Modules
{
    TestModule::TestModule()
    {
        motor1 = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::shooterID, "Default Name");

        motor1->GetConfigurator().Apply(Constants::Shooter::shooterMotorCondiguration);
    }

    TestModule::~TestModule()
    {
        delete(motor1);
    }

    void TestModule::Shoot()
    {
        motor1->Set(1);
    }

    void TestModule::Stop()
    {
        motor1->Set(0);
    }
}