#include "Modules/NetworkTableModule.h"

#include <iostream>

namespace Modules
{
       
    NetworkTableModule::NetworkTableModule()
    {
        nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
        
        inst.SetServerTeam(2352);
        inst.StartClient4("MyConsoleClient");
        
        std::shared_ptr<nt::NetworkTable> table = inst.GetTable("SmartDashboard");

        
        table = inst.GetTable("limelight-main");

        testEntry = table->GetEntry("tv");
        test2Entry = table->GetEntry("Test2");
    }
    
    void NetworkTableModule::SetUsedPrograms(Programs usedPrograms)
    {
        this->usedPrograms = usedPrograms;
    }

    void NetworkTableModule::Update()
    {
        test2Entry.SetDouble(10.0);
    }

    void NetworkTableModule::PIDProgram()
    {
    }

    void NetworkTableModule::ShooterTunningProgram()
    {
    }
}