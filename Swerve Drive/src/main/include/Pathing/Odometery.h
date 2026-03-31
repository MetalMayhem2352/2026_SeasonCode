#pragma once

#include "Pathing/TunerConstants.h"

#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "ctre/phoenix6/Pigeon2.hpp"
#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"

namespace Pathing
{
    class Odometry
    {
    public:
        Odometry(TunerSwerveDrivetrain* drivetrain,
            ctre::phoenix6::hardware::Pigeon2* pigeon);

        void Update();  
        frc::Pose2d GetPose() const;
        void ResetPose(const frc::Pose2d& pose);

    private:
        // Core references
        TunerSwerveDrivetrain* m_drivetrain;

        ctre::phoenix6::hardware::Pigeon2* m_pigeon;

        // Limelight tables
        std::shared_ptr<nt::NetworkTable> m_ll4;   // limelight-main
        // std::shared_ptr<nt::NetworkTable> m_ll3;   // limelight-left

        // Internal helpers
        void ProcessLimelight(std::shared_ptr<nt::NetworkTable> table, bool isLL4);
        void AutoInitializePose();

        // Dynamic trust control
        void UpdateVisionTrust();

        // State
        frc::Pose2d m_currentPose;
        double m_lastBumpTime = 0.0;

        // Vision trust settings
        struct VisionStdDevs {
            double x;
            double y;
            double theta;
        };

        VisionStdDevs m_ll4StdDevNormal{0.3, 0.3, 8.0};
        VisionStdDevs m_ll3StdDevNormal{0.6, 0.6, 12.0};

        VisionStdDevs m_highTrust{0.1, 0.1, 5.0};
    };
}