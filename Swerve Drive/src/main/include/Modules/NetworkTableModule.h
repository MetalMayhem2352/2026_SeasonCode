#pragma once

#include "Core/PiecewiseLinearFunctionXYZ.h"

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/BooleanTopic.h>


namespace Modules
{
    class NetworkTableModule
    {
        public:
            enum Programs
            {
                PIDTunning = 0,
                ShooterTunning = 1 << 0,
                AutoNavigation = 1 << 2,
                TeleopTracking = 1 << 3,
                GameManagment = 1 << 4
            };

        private:
            Programs usedPrograms;

            std::shared_ptr<nt::NetworkTable> table;
            nt::NetworkTableEntry distanceEntery;
            nt::NetworkTableEntry powerEntery;
            nt::NetworkTableEntry hoodEntery;
            nt::NetworkTableEntry saveEntery;

            Core::PiecewiseLinearFunctionXYZ shooingDistanceTable;
        public:
            NetworkTableModule();

            void SetUsedPrograms(Programs usedPrograms);

            void Update();

        private:
            
            void PIDProgram(); 
            void ShooterTunningProgram(); 
            
    };
}