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
#include <frc/Compressor.h>
#include <frc/Talon.h>
#include <Solenoid.h>
#include <DoubleSolenoid.h>
#include <math.h>
using namespace frc;
frc::Joystick stick{0};
frc::Talon Left{0},Right{1};
rev::SparkMax wheel{2},intake{3},outtake{4};
frc::DifferentialDrive myRobot{Left, Right};
frc::Timer timer;

//frc::SendableChooser autoChoice;
Solenoid piston1{0};
Solenoid piston2{1};
Compressor compressor{0};
double speed, turn, sensitivity, turnKey;
bool isUpPressed, isDownPressed;
double sP,tN;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  compressor.SetClosedLoopControl(false);
  compressor.Start();
  timer.Reset();
  timer.Start();
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
  /*
  piston.Set(true); makes the piston go
  piston.Set(false); makes the piston not go
  */
  if (stick.GetRawButton(6) && sensitivity < 1.0) {
    sensitivity += 0.01;
  }
  else if (stick.GetRawButton(6)) {
    sensitivity += 0;
  }
  else if (stick.GetRawButton(5) && sensitivity > 0.0) {
    sensitivity -= 0.01;
  }
  else if (stick.GetRawButton(5)) {
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
  if (stick.GetRawButton(5)) {
    wheel.Set(0.3);
  }else{
    wheel.Set(0);

  }
  if (stick.GetRawButton(2)) {
    intake.Set(stick.GetRawAxis(1));
  }else{
    intake.Set(0);
  }
  //turn with bumpers, too jittery
  /*if(stick.GetRawButton(7)){
     turn = (-1 * sensitivity);
  } else if(stick.GetRawButton(8)){
     turn = (1 * sensitivity);
  } else{
    turn = 0;
  }
  */
  if(stick.GetRawAxis(4)>0.2||stick.GetRawAxis(4)<-0.2){
			tN=stick.GetRawAxis(4);
		}else{
      tN=0;
    }
  if(stick.GetRawAxis(1)>0.2||stick.GetRawAxis(1)<-0.2){
			sP=stick.GetRawAxis(1);
  }else{
    sP=0;
  }
  speed = -sP * sensitivity;
  if (speed >= 0) {
    turn = ((tN * sensitivity)+(speed/4))+0.1;
  }
  else {
    turn = ((tN*sensitivity)-(speed/4))+0.15;
  }
  myRobot.ArcadeDrive(speed, -turn);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
