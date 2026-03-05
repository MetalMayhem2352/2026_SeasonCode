#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

namespace CustomSwerveDrive
{
    class SwerveEncoder
    {
    public:
        SwerveEncoder(int enocderIndex);
        ~SwerveEncoder();

        void SetInverted(bool isInverted);
        void ResetAngle();
        void SetOffset(double offset);
        double GetAngle();
    private:
        ctre::phoenix6::hardware::CANcoder* encoder;
        double position;
        double positionOffset;
        bool isInverted;
    };
}
