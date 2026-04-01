#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace Modules
{
	class NewTurretModule 
	{
        public:
            enum State
            {
                Shoot,
                PassLeft,
                PassRight,
            };

		private:
            State currentState;

			frc::Timer timer;

			Core::Timer* pidTimer;
			Core::PIDController* PIDController;

			double tx;
			bool hasTarget;

			ctre::phoenix6::hardware::TalonFX* turret_motor; 
			
		public:
			NewTurretModule();
			~NewTurretModule();

			void Update(frc::Pose2d robotPosition, frc::ChassisSpeeds velocity, double xInput, double zInput, double rotationInput);
			void UpdateState(State newState);
            
            bool CanShoot();
            double GetTurretPosition();
        private:
			void Rotate(double angle);
			double GetTargetPosition(frc::Pose2d targetPosition, frc::Pose2d robotPosition, frc::ChassisSpeeds velocity, double xInput, double zInput, double rotationInput);
	};
}