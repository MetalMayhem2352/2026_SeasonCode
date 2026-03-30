#pragma once

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


        public:

            void SetUsedPrograms(Programs usedPrograms);

            void Update();

        private:
            
            void ShooterTunningProgram(); 
            
    };
}