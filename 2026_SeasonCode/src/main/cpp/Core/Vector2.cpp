#include "Core/Vector2.h"
#include "Constants.h"

#include <cmath>
#include <iostream>

namespace Core
{
    Vector2::Vector2()
    {
        x = 0;
        y = 0;
    }
    Vector2::Vector2(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
    Vector2::~Vector2()
    {

    }
    /// @brief Creates a vector that points in a direction with a set madniguted, USES DEGREES
    /// @param angle is the amount of degrees clockwise that the vector is rotated, 0 being north.
    /// @param magnitude the length of hte vecotr
    /// @return The created Vector
    Vector2 Vector2::CreateAngularVector(double angle, double magnitude)
    {
        double angleInRadians = Constants::DEGREES_TO_RADIANS * angle;
        
        double x = magnitude * std::sin(angleInRadians);
        double y = magnitude * std::cos(angleInRadians);
        
        return Vector2(x, y);
    }


    [[nondiscard]] double Vector2::GetX() const
    {
        return x;
    }
    [[nondiscard]] double Vector2::GetY() const
    {
        return y;
    }
    /// @brief Gets the rotation of the vector in Degrees With 0 pointing north
    /// @return rotation of the vector in Degrees
    [[nondiscard]] double Vector2::GetAngle() const
    {
        double angle = (std::atan2(y, x) * Constants::RADIANS_TO_DEGREES);
        while (angle < 360)
        {
            angle += 360;
        }

        /*
        // Might work?
        if (y == 0)
        {
            angle = 90;
        }
        */

        return angle;
    }
    [[nondiscard]] double Vector2::GetMagnitude() const
    {
        return std::sqrt(x * x + (y * y)); 
    }



    void Vector2::Set(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
    void Vector2::SetX(double x)
    {
        this->x = x;
    }
    void Vector2::SetY(double y)
    {
        this->y = y;
    }
    /// @brief Sets the angle the vector in Degrees With 0 pointing north
    /// @param degrees The amount of degrees the vector will be rotated from true north
    void Vector2::SetAngle(double degrees)
    {
        double magnitude = GetMagnitude();
        double angleInRadians = Constants::DEGREES_TO_RADIANS * degrees;
        
        x = magnitude * std::sin(angleInRadians);
        y = magnitude * std::cos(angleInRadians);
    }
    void Vector2::SetMagnitude(double magnitude)
    {
        if (x != 0 || y != 0)
        {
            double quotient = magnitude / GetMagnitude();
            x *= quotient;
            y *= quotient;
        }
    }


    // ================================ Operator overloads ================================ 
    [[nondiscard]] Vector2 Vector2::operator+(const Vector2& other) const
    {
        return Vector2(x + other.x, y + other.y);
    }
    Vector2& Vector2::operator+=(const Vector2& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }
    
    [[nondiscard]] Vector2 Vector2::operator-(const Vector2& other) const
    {
        return Vector2(x - other.x, y - other.y);
    }
    Vector2& Vector2::operator-=(const Vector2& other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    
    [[nondiscard]] Vector2 Vector2::operator*(const Vector2& other) const
    {
        return Vector2(x * other.x, y * other.y); // component-wise multiply
    }
    Vector2& Vector2::operator*=(const Vector2& other)
    {
        // component-wise multiply
        x *= other.x;
        y *= other.y;
        return *this;
    }
    
    [[nondiscard]] Vector2 Vector2::operator/(const Vector2& other) const
    {
        if (other.x == 0.0 || other.y == 0.0) 
        {
            std::cout << "ERROR! DIVISION BY ZERO!\n";
            return Vector2(x, y);
        }
        return Vector2(x / other.x, y / other.y);
    }
    Vector2& Vector2::operator/=(const Vector2& other)
    {
        if (other.x == 0.0 || other.y == 0.0) 
        {
            std::cout << "ERROR! DIVISION BY ZERO!\n";
            return *this;
        }
        x /= other.x;
        y /= other.y;
        return *this;

    }
    
    [[nondiscard]] Vector2 Vector2::operator%(const Vector2& other) const
    {
        if (other.x == 0.0 || other.y == 0.0) 
        {
            std::cout << "ERROR! MODULUS BY ZERO!\n";
            return Vector2(x, y);
        }
        return Vector2(std::fmod(x, other.x), std::fmod(y, other.y));

    }
    Vector2& Vector2::operator%=(const Vector2& other)
    {
        if (other.x == 0.0 || other.y == 0.0) 
        {
            std::cout << "ERROR! MODULUS BY ZERO!\n";
            return *this;
        }
        x = std::fmod(x, other.x);
        y = std::fmod(y, other.y);
        return *this;
    }

    // ================= Scalar overloads =================
    [[nondiscard]] Vector2 Vector2::operator+(double scalar) const 
    {
        return Vector2(x + scalar, y + scalar);
    }
    Vector2& Vector2::operator+=(double scalar) 
    {
        x += scalar;
        y += scalar;
        return *this;
    }
    
    [[nondiscard]] Vector2 Vector2::operator-(double scalar) const 
    {
        return Vector2(x - scalar, y - scalar);
    }
    Vector2& Vector2::operator-=(double scalar) 
    {
        x -= scalar;
        y -= scalar;
        return *this;
    }
    
    [[nondiscard]] Vector2 Vector2::operator*(double scalar) const 
    {
        return Vector2(x * scalar, y * scalar);
    }
    Vector2& Vector2::operator*=(double scalar) 
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }
    
    [[nondiscard]] Vector2 Vector2::operator/(double scalar) const 
    {
        if (scalar == 0.0) 
        {
            std::cout << "ERROR! DIVISION BY ZERO!\n";
            return Vector2(x, y);
        }
        return Vector2(x / scalar, y / scalar);
    }
    Vector2& Vector2::operator/=(double scalar) 
    {
        if (scalar == 0.0) 
        {
            std::cout << "ERROR! DIVISION BY ZERO!\n";
            return *this;
        }
        x /= scalar;
        y /= scalar;
        return *this;
    }

    [[nondiscard]] Vector2 Vector2::operator%(double scalar) const 
    {
        if (scalar == 0.0) 
        {
            std::cout << "ERROR! MODULUS BY ZERO!\n";
            return Vector2(x, y);
        }
        return Vector2(std::fmod(x, scalar), std::fmod(y, scalar));
    }
    Vector2& Vector2::operator%=(double scalar) 
    {
        if (scalar == 0.0) 
        {
            std::cout << "ERROR! MODULUS BY ZERO!\n";
            return *this;
        }
        x = std::fmod(x, scalar);
        y = std::fmod(y, scalar);
        return *this;
    }
    
}