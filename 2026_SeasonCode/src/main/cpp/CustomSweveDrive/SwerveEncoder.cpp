#include "CustomSwerveDrive/SwerveEncoder.h"

#include "Constants.h"

// TEMP!
#include <iostream>>

namespace CustomSwerveDrive
{
    SwerveEncoder::SwerveEncoder(char enocderIndex)
        : encoder{enocderIndex, Constants::CANIVOUR_NAME}
    {
    }

    SwerveEncoder::~SwerveEncoder()
    {
        
    }


    void SwerveEncoder::SetInverted(bool isInverted)
    {
        this->isInverted = isInverted;
    }

    void SwerveEncoder::SetOffset(double offset)
    {
        positionOffset = offset / 360;
    }

    void SwerveEncoder::ResetAngle()
    {
        positionOffset = encoder.GetAbsolutePosition().GetValueAsDouble();
    }

    double SwerveEncoder::GetAngle()
    {
        std::cout << "POSITION: " << encoder.GetAbsolutePosition().GetValueAsDouble() << '\n';
        position = (encoder.GetAbsolutePosition().GetValueAsDouble() - positionOffset) * 360;
        
        
        // normallizing Point to 0-360;
        while (position < 0)
        {
            position += 360;
        }
        while (position > 360)
        {
            position -= 360;
        }

        if (isInverted)
        {
            position = 360 -position;
        }

        return position;
    }
}
