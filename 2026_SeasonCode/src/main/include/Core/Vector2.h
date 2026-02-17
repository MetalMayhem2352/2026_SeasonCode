#pragma once
#include <numbers>

namespace Core
{
    class Vector2
    {
    private:
        double x;
        double y;
    public:
        Vector2();
        Vector2(double x, double y);
        ~Vector2();
        static Vector2 CreateAngluarVector(double angle, double magnitude);

        double GetX() const;
        double GetY() const;
        double GetAngle() const;
        double GetMagnitude() const; 

        void Set(double x, double y);
        void SetX(double x);
        void SetY(double y);
        void SetAngle(double degrees);
        void SetMagnitude(double magnitude);

        // ================================ Operator overloads ================================ 
        Vector2 operator+(const Vector2& other) const;
        Vector2& operator+=(const Vector2& other);
        
        Vector2 operator-(const Vector2& other) const;
        Vector2& operator-=(const Vector2& other);

        Vector2 operator/(const Vector2& other) const;
        Vector2& operator/=(const Vector2& other);
    
        Vector2 operator*(const Vector2& other) const;
        Vector2& operator*=(const Vector2& other);
    
        Vector2 operator%(const Vector2& other) const;
        Vector2& operator%=(const Vector2& other);
    

        // ================= Scalar overloads =================
        Vector2 operator+(double other) const;
        Vector2& operator+=(double other);
        
        Vector2 operator-(double other) const;
        Vector2& operator-=(double other);
        
        Vector2 operator/(double other) const;
        Vector2& operator/=(double other);
        
        Vector2 operator*(double other) const;
        Vector2& operator*=(double other);
        
        Vector2 operator%(double other) const;
        Vector2& operator%=(double other);
    };
}