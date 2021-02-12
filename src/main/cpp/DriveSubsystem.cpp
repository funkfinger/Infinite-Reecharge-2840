#include "subsystems/DriveSubsystem.h"

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

using namespace Constants;


DriveSubsystem::DriveSubsystem() {
  ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_left1 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
  ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_left2 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
  ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_right1 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(3);
  ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_right2 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
  //   m_leftEncoder{kLeftEncoderPorts[0], kLeftEncoderPorts[1]},
  //   m_rightEncoder{kRightEncoderPorts[0], kRightEncoderPorts[1]},
  //m_odometry.ResetPosition(frc::Pose2d(), toRotation(m_gyro.GetAbsoluteCompassHeading()));
  // m_odometry = frc::DifferentialDriveOdometry(toRotation(m_gyro.GetAbsoluteCompassHeading()), frc::Pose2d());
  // m_odometry{toRotation(m_gyro.GetAbsoluteCompassHeading())} {
  //   ResetEncoders();
  // };
  // Set the distance per pulse for the encoders
//   m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
//   m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(toRotation(m_gyro.GetAbsoluteCompassHeading()),
                    //units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t((*m_left1).GetSelectedSensorPosition()),
                    //units::meter_t(m_rightEncoder.GetDistance()));
                    units::meter_t((*m_right1).GetSelectedSensorPosition()));
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(-right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
//   m_leftEncoder.Reset();
//   m_rightEncoder.Reset();
    m_left1->SetSelectedSensorPosition(0.0);
    m_right1->SetSelectedSensorPosition(0.0);
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (m_left1->GetSelectedSensorPosition() + m_right1->GetSelectedSensorPosition()) / 2.0;
}

// frc::Encoder& DriveSubsystem::GetLeftEncoder() {
//   return m_leftEncoder;
// }

// frc::Encoder& DriveSubsystem::GetRightEncoder() {
//   return m_rightEncoder;
// }

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() {
  double deg = m_gyro.GetAbsoluteCompassHeading();
  units::degree_t heading = units::degree_t(deg);
  return units::degree_t(heading);
}

double DriveSubsystem::GetTurnRate() {
  double xyz_dps[3];
  m_gyro.GetRawGyro(xyz_dps);
  return -xyz_dps[2];
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t((*m_left1).GetSelectedSensorVelocity()),
          units::meters_per_second_t((*m_right1).GetSelectedSensorVelocity())};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, toRotation(m_gyro.GetAbsoluteCompassHeading()));
}