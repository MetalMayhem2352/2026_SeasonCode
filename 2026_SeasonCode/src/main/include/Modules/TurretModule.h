
#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>
#include "Core/Timer.h"
#include "Core/PIDController.h"

class Turret_Tracking {
    
  frc::Timer timer;

  Core::Timer* PIDTimer;
  Core::PIDController* PIDController;

  double tx;

  double forward = 0;
  double backward = 0;
  double currentpos;
  double motorangle;
  double angleoffset;
  double error;

  double maxRotation;
  double minRotation;
  bool hasTarget;

  ctre::phoenix6::hardware::TalonFX turret_motor{1};

  public:
  Turret_Tracking();
  ~Turret_Tracking();

  int Find_april();
  int Track();
  void Update();
   

};