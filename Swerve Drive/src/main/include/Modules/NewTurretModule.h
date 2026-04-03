#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

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

			double taregtDistance = 0;

			ctre::phoenix6::hardware::TalonFX* turret_motor; 
			
			// PID Write
			nt::NetworkTableEntry kP_Entry;
			nt::NetworkTableEntry kPDeadzone_Entry;
			nt::NetworkTableEntry kI_Entry;
			nt::NetworkTableEntry kIActiveZone_Entry;
			nt::NetworkTableEntry kD_Entry;

			// Turret Positioning Read Write
			nt::NetworkTableEntry minAngleEntry;
			nt::NetworkTableEntry maxAngleEntry;

			nt::NetworkTableEntry targetAngleEntry;

			nt::NetworkTableEntry currentPositionEntry;

			// Shoot and move
			nt::NetworkTableEntry currentVelocityModifierEntry;
			nt::NetworkTableEntry predictedVelocityModifierEntry;

			nt::NetworkTableEntry saveEntry;

		public:
			NewTurretModule();
			~NewTurretModule();

			void Update(frc::Pose2d robotPosition, frc::ChassisSpeeds velocity, double xInput, double zInput, double rotationInput);
			void UpdateState(State newState);
            
            bool CanShoot();
			double GetTargetDistance();
			
			void Rotate(double angle);
		private:
            
			double GetTurretPosition();
			double GetTargetPosition(frc::Pose2d targetPosition, frc::Pose2d robotPosition, frc::ChassisSpeeds velocity, double xInput, double zInput, double rotationInput);
			void CalculateTargetDistance(frc::Pose2d targetPosition, frc::Pose2d robotPosition, frc::ChassisSpeeds velocity, double xInput, double zInput, double rotationInput);
	};
}