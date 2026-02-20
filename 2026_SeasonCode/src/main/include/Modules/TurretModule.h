
#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

class Turret_Tracking {
    
  frc::Timer timer;

  double tx;

  double forward = 0;
  double backward = 0;

  double motorangle;

  double maxRotation;
  double minRotation;
  bool hasTarget;

  ctre::phoenix6::hardware::TalonFX turret_motor{1};

  public:
  Turret_Tracking();

   int Calibrate();
   int find_april();
   int track();
   

};