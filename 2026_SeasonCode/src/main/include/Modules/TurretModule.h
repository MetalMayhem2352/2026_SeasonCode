#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"
#include "CustomSwerveDrive/SwerveDriveModule.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

class Turret_Tracking 
{
	private:
		frc::Timer timer;

		Core::Timer* pidTimer;
		Core::PIDController* PIDController;

		double maxRotation;
		double minRotation;

		ctre::phoenix6::hardware::TalonFX* turret_motor; 

		CustomSwerveDrive::SwerveDriveModule* swerveDrive; 
		
	public:
		Turret_Tracking(CustomSwerveDrive::SwerveDriveModule* swerveDrive);
		~Turret_Tracking();

		void Find_april();
		void Track();
		void Update();
		void turretIdle();
		double limelight_Distance();
		bool CanShoot();

		// void Rotate(double angle);
		// double GetTurretPosition();
// 
		// void Pass();
		// bool CanPass();
};