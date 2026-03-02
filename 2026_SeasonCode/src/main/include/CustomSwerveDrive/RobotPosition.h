#include "Core/Vector2.h"

namespace CustomDriveBase
{
    // Robot Position on the field in Meters
    class RobotPosition
    {
    public:

        double x;
        double z;
        double heading;

        
        RobotPosition operator+(const Core::Vector2& other) const
        {
            RobotPosition newRobotPosition(*this);
            newRobotPosition.x += other.GetX();
            newRobotPosition.z += other.GetY();
        }
        RobotPosition& operator+=(const Core::Vector2& other)
        {
            x += other.GetX();
            z += other.GetY();
        }
        
        RobotPosition operator-(const Core::Vector2& other) const
        {
            RobotPosition newRobotPosition(*this);
            newRobotPosition.x -= other.GetX();
            newRobotPosition.z -= other.GetY();
        }
        RobotPosition& operator-=(const Core::Vector2& other)
        {
            x -= other.GetX();
            z -= other.GetY();
        }
    };
}