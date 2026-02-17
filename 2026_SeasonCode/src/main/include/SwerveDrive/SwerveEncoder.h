#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

namespace SwerveDrive
{
    class SwerveEncoder
    {
    public:
        SwerveEncoder(char enocderIndex);
        ~SwerveEncoder();

        void SetInverted(bool isInverted);
        void ResetAngle();
        void SetOffsert(double offset);
        double GetAngle();
    private:
        ctre::phoenix6::hardware::CANcoder encoder;
        double position;
        double positionOffset;
        bool isInverted;
    };
}
