/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <memory>
#include <string>

#include <frc/IterativeRobot.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <frc/WPILib.h>
#include <frc/PowerDistributionPanel.h>
#include <cameraServer/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SmartDashboard/SendableChooser.h>

#include "rev/SparkMax.h"
#include <frc/Talon.h>

#include <math.h>

frc::Talon frontRight{12}, backRight{14}, backLeft{0};
frc::Joystick stick{0};
rev::SparkMax frontLeft{0};
frc::RobotDrive myRobot{frontLeft, frontRight, backLeft, backRight};
frc::Timer timer;
//frc::SendableChooser autoChoice;

double speed, turn, sensitivity;
bool isUpPressed, isDownPressed;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  timer.Reset();
  timer.Start();
  turn = 0;
  speed = 0;
  sensitivity = 0.5;
}

void Robot::TeleopPeriodic() {
  //increase sensitivity with the right bumper
  if (stick.GetRawButton(2) && sensitivity < 1.0) {
    sensitivity += 0.01;
  }
  else if (stick.GetRawButton(2)) {
    sensitivity += 0;
  }
  else if (stick.GetRawButton(3) && sensitivity > 0.0) {
    sensitivity -= 0.01;
  }
  else if (stick.GetRawButton(3)) {
    sensitivity -= 0;
  }
  else {
    sensitivity = sensitivity;
  }
  if (sensitivity >= 1.0) {
    sensitivity = 1.0;
  }
  else if (sensitivity <= 0) {
    sensitivity = 0.0;
  }
  else {}
  //drive with the left joystick
  turn = stick.GetRawAxis(0) * 0.95;
  speed = -stick.GetRawAxis(1) * sensitivity;
  myRobot.ArcadeDrive(speed, turn);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
