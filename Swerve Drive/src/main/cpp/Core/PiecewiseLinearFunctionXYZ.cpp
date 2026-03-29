#include "Core/PiecewiseLinearFunctionXYZ.h"

#include <wpi/json.h>
#include <fstream>

namespace Core
{
    PiecewiseLinearFunctionXYZ::PiecewiseLinearFunctionXYZ(const std::vector<Point>& pts)
        : points(pts)
    {
        Sort();
    }

    void PiecewiseLinearFunctionXYZ::AddPoint(float x, float y, float z) {
        points.push_back({ x, y, z });
        Sort();
    }

    bool PiecewiseLinearFunctionXYZ::EditPoint(size_t index, float newX, float newY, float newZ) {
        if (index >= points.size())
            return false;

        points[index] = { newX, newY, newZ };
        Sort();
        return true;
    }

    bool PiecewiseLinearFunctionXYZ::RemovePoint(size_t index) {
        if (index >= points.size())
            return false;

        points.erase(points.begin() + index);
        return true;
    }

    void PiecewiseLinearFunctionXYZ::Clear() {
        points.clear();
    }

    size_t PiecewiseLinearFunctionXYZ::Size() const {
        return points.size();
    }

    PiecewiseLinearFunctionXYZ::Output PiecewiseLinearFunctionXYZ::Get(float x) const {
        if (points.empty())
            return { 0.0f, 0.0f };

        if (x <= points.front().x)
            return { points.front().y, points.front().z };

        if (x >= points.back().x)
            return { points.back().y, points.back().z };

        for (size_t i = 0; i < points.size() - 1; i++) {
            const Point& p0 = points[i];
            const Point& p1 = points[i + 1];

            if (x >= p0.x && x <= p1.x) {
                float t = (x - p0.x) / (p1.x - p0.x);

                float y = p0.y + t * (p1.y - p0.y);
                float z = p0.z + t * (p1.z - p0.z);

                return { y, z };
            }
        }

        return { 0.0f, 0.0f };
    }

    bool PiecewiseLinearFunctionXYZ::SaveToFile(const std::string& path) const {
        wpi::json json;

        for (const auto& point : points) {
            json["points"].push_back({
                {"x", point.x},
                {"y", point.y},
                {"z", point.z}
            });

        }

        std::ofstream file(path);
        if (!file.is_open())
            return false;

        file << json.dump(4);
        return true;
    }

    bool PiecewiseLinearFunctionXYZ::LoadFromFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open())
            return false;

        wpi::json json;
        file >> json;

        points.clear();

        for (auto& item : json["points"]) {
            points.push_back({
                item["x"],
                item["y"],
                item["z"]
            });
        }

        Sort();
        return true;
    }

    void PiecewiseLinearFunctionXYZ::Sort() {
        // Sort by X
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