#pragma once
#include <vector>
#include <algorithm>

#include <string>

namespace Core
{
    class PiecewiseLinearFunction {
    public:
        struct Point {
            float x;
            float y;
        };


    private:
        std::vector<Point> points;

    public:

        // Constructor: optional initial points
        PiecewiseLinearFunction(const std::vector<Point>& pts = {});

        // Add a point at runtime
        void AddPoint(float x, float y);

        // Edit an existing point by index
        bool EditPoint(size_t index, float newX, float newY);

        // Remove a point by index
        bool RemovePoint(size_t index);

        // Clear all points
        void Clear();

        // Evaluate the function at input x
        float Get(float x) const;

        // Number of points
        size_t Size() const;

        // Saves to a location (use /home/lvuser/FILENAME)
        bool SaveToFile(const std::string& path) const;

        // Loads from a location (use /home/lvuser/FILENAME)
        bool LoadFromFile(const std::string& path);


    private:
        void Sort();
    };
}