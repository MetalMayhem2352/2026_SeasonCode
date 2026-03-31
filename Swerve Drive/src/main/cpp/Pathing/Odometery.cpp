#include "Pathing/Odometery.h"


#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>

namespace Pathing
{
    Odometry::Odometry(
        TunerSwerveDrivetrain* drivetrain,
        ctre::phoenix6::hardware::Pigeon2* pigeon)
        : m_drivetrain(drivetrain),
        m_pigeon(pigeon)
    {
        m_ll4 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-main");
        m_ll3 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");

        m_currentPose = frc::Pose2d();
    }

    void Odometry::Update()
    {
        // Update CTRE odometry
        m_currentPose = m_drivetrain->GetState().Pose;

        // Auto initialize pose if possible
        AutoInitializePose();

        // Adjust trust dynamically
        UpdateVisionTrust();

        // Process both cameras
        ProcessLimelight(m_ll4, true);
        ProcessLimelight(m_ll3, false);
    }

    void Odometry::AutoInitializePose()
    {
        static bool initialized = false;
        if (initialized) return;

        auto arr = m_ll4->GetNumberArray("botpose_wpiblue", {});
        if (arr.size() < 7) return;

        if (arr[0] == 0.0 && arr[1] == 0.0) return;

        frc::Pose2d pose{
            units::meter_t(arr[0]),
            units::meter_t(arr[1]),
            frc::Rotation2d(units::degree_t(arr[5]))
        };

        m_drivetrain->ResetPose(pose);
        initialized = true;
    }

    void Odometry::UpdateVisionTrust()
    {
        double pitch = m_pigeon->GetPitch().GetValueAsDouble();
        double roll  = m_pigeon->GetRoll().GetValueAsDouble();

        bool onBump = (std::abs(pitch) > 10.0 || std::abs(roll) > 10.0);
        double now = frc::Timer::GetFPGATimestamp().value();

        if (onBump) {
            m_lastBumpTime = now;
            m_drivetrain->SetVisionMeasurementStdDevs(
                {m_highTrust.x, m_highTrust.y, m_highTrust.theta});
        } else {
            if (now - m_lastBumpTime > 1.0) {
                // Back to normal trust (LL4 default)
                m_drivetrain->SetVisionMeasurementStdDevs(
                    {m_ll4StdDevNormal.x, m_ll4StdDevNormal.y, m_ll4StdDevNormal.theta});
            }
        }
    }

    void Odometry::ProcessLimelight(std::shared_ptr<nt::NetworkTable> table, bool isLL4)
    {
        double tv = table->GetNumber("tv", 0.0);
        if (tv < 1.0) return;

        auto botpose = table->GetNumberArray("botpose_wpiblue", {});
        if (botpose.size() < 7) return;

        frc::Pose2d visionPose{
            units::meter_t(botpose[0]),
            units::meter_t(botpose[1]),
            frc::Rotation2d(units::degree_t(botpose[5]))
        };

        double latencyMs = botpose[6];
        double timestamp = frc::Timer::GetFPGATimestamp().value() - latencyMs / 1000.0;

        // Apply per-camera trust
        if (isLL4) {
            m_drivetrain->SetVisionMeasurementStdDevs(
                {m_ll4StdDevNormal.x, m_ll4StdDevNormal.y, m_ll4StdDevNormal.theta});
        } else {
            m_drivetrain->SetVisionMeasurementStdDevs(
                {m_ll3StdDevNormal.x, m_ll3StdDevNormal.y, m_ll3StdDevNormal.theta});
        }

        m_drivetrain->AddVisionMeasurement(visionPose, units::second_t(timestamp));
    }

    frc::Pose2d Odometry::GetPose() const
    {
        return m_currentPose;
    }

    void Odometry::ResetPose(const frc::Pose2d& pose)
    {
        m_drivetrain->ResetPose(pose);
        m_currentPose = pose;
    }
}