#include "Core/PiecewiseLinearFunction.h"

#include <wpi/json.h>
#include <fstream>

namespace Core
{
    PiecewiseLinearFunction::PiecewiseLinearFunction(const std::vector<Point>& pts)
        : points(pts)
    {
        Sort();
    }

    void PiecewiseLinearFunction::AddPoint(float x, float y) {
        points.push_back({ x, y });
        Sort();
    }

    bool PiecewiseLinearFunction::EditPoint(size_t index, float newX, float newY) {
        if (index >= points.size())
            return false;

        points[index] = { newX, newY };
        Sort();
        return true;
    }

    bool PiecewiseLinearFunction::RemovePoint(size_t index) {
        if (index >= points.size())
            return false;

        points.erase(points.begin() + index);
        return true;
    }

    void PiecewiseLinearFunction::Clear() {
        points.clear();
    }

    size_t PiecewiseLinearFunction::Size() const {
        return points.size();
    }

    float PiecewiseLinearFunction::Get(float x) const {
        if (points.empty())
            return 0.0f;

        if (x <= points.front().x)
            return points.front().y;

        if (x >= points.back().x)
            return points.back().y;

        for (size_t i = 0; i < points.size() - 1; i++) {
            const Point& point0 = points[i];
            const Point& point1 = points[i + 1];

            if (x >= point0.x && x <= point1.x) {
                float t = (x - point0.x) / (point1.x - point0.x);
                return point0.y + t * (point1.y - point0.y);
            }
        }

        return 0.0f;
    }

    bool PiecewiseLinearFunction::SaveToFile(const std::string& path) const {
        wpi::json json;

        for (const auto& point : points) {
            json["points"].push_back({ {"x", point.x}, {"y", point.y} });
        }

        std::ofstream file(path);
        if (!file.is_open())
            return false;

        file << json.dump(4);
        return true;
    }

    bool PiecewiseLinearFunction::LoadFromFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open())
            return false;

        wpi::json json;
        file >> json;

        points.clear();

        for (auto& item : json["points"]) {
            points.push_back({ item["x"], item["y"] });
        }

        Sort();
        return true;
    }

    void PiecewiseLinearFunction::Sort() {
        // Sort by x
        std::sort(points.begin(), points.end(),
            [](const Point& a, const Point& b) {
                return a.x < b.x;
            });

        // Remove duplicate x values
        auto newEnd = std::unique(points.begin(), points.end(),
            [](const Point& a, const Point& b) {
                return a.x == b.x;
            });

        points.erase(newEnd, points.end());
    }
}