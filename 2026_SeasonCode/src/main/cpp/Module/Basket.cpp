#include "Modules/Basket.h"
#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

Basket::Basket(){
    double minHeight = Basket_motor_1.GetPosition().GetValue().value();
    double maxHeight = 0;
}
