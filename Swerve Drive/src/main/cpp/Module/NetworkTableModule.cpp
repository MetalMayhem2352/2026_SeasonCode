#include "Modules/NetworkTableModule.h"

#include "Constants.h"

#include <iostream>

namespace Modules
{
       
    NetworkTableModule::NetworkTableModule()
        // :shooingDistanceTable({{0, 0, 0}})
    {
        // shooingDistanceTable.SaveToFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);

        shooingDistanceTable.LoadFromFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);


        nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
        
        inst.SetServerTeam(2352);
        inst.StartClient4("MyConsoleClient");
        
        std::shared_ptr<nt::NetworkTable> table = inst.GetTable("SmartDashboard");
        tablePointer = table.get();

        distanceEntery = table->GetEntry("distance");
        powerEntery = table->GetEntry("power");
        hoodEntery = table->GetEntry("hood");
        saveEntery = table->GetEntry("save");
    }
    
    void NetworkTableModule::SetUsedPrograms(Programs usedPrograms)
    {
        this->usedPrograms = usedPrograms;
    }

    void NetworkTableModule::Update()
    {
        if (saveEntery.GetBoolean(false) == true)
        {
            saveEntery.SetBoolean(false);

            double hoodPos = hoodEntery.GetDouble(1);

            shooingDistanceTable.AddPoint(distanceEntery.GetDouble(-1), powerEntery.GetDouble(0), hoodPos);
            shooingDistanceTable.SaveToFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);
        }
    }
    
    nt::NetworkTable* NetworkTableModule::GetNetworkTable()
    {
        return tablePointer;
    }

    void NetworkTableModule::PIDProgram()
    {
    }

    void NetworkTableModule::ShooterTunningProgram()
    {
    }
}