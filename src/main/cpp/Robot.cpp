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
#include <array>

#include <frc/IterativeRobot.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/Spark.h>
#include <frc/Talon.h>
#include <frc/Encoder.h>
#include <frc/WPILib.h>
#include <frc/PowerDistributionPanel.h>
#include <cameraServer/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/Servo.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "rev/SparkMax.h"
#include <frc/Compressor.h>
#include <frc/Talon.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AddressableLED.h>
#include <math.h>

cs::UsbCamera camera0;
cs::UsbCamera camera1;
cs::VideoSink server;
frc::Joystick one{0}, two{1};
//rev::SparkMax intake{4}, outtake{5};
rev::SparkMax in{5}, intake{4};
frc::Servo pan{6},tilt{7};
frc::Talon frontLeft{2}, frontRight{0}, backRight{3}, backLeft{1}, out{8};

// ctre::phoenix::motorcontrol::can::WPI_TalonSRX *frontLeft = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(2);
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX *frontRight = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(1);
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX *backLeft= new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(3);
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX *backRight = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(0);
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX *panel = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(10);

//ctre::phoenix::motorcontrol::can::WPI_TalonFX::WPI_TalonFX * frontRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX::WPI_TalonFX(1);
// WPI_TalonFX * _rghtFollower = new WPI_TalonFX(3);
// WPI_TalonFX * _leftFront = new WPI_TalonFX(2);
// WPI_TalonFX * _leftFollower = new WPI_TalonFX(4);
//ctre::phoenix::motorcontrol::can::VictorSPX frontLeft{2}, frontRight{1}, backLeft{3}, backRight{0}, panel{10};
frc::RobotDrive myRobot{frontLeft, backLeft, frontRight, backRight};
frc::Timer timer, shootTimer;


//frc::SendableChooser autoChoice;
frc::Solenoid ballUnstuck{0};
frc::DoubleSolenoid ballIn{3, 4}, ballStorage{2, 1};
frc::Compressor *compressor = new frc::Compressor(0);

ctre::phoenix::sensors::PigeonIMU pigeon{10};

double speed, turn, sensitivity = 0.65, turnKey;
bool isUpPressed, isDownPressed;
double sP,tN;
int16_t accel[3];

static constexpr int kLength = 278;

// PWM port 9
// Must be a PWM header, not MXP or DIO
//frc::AddressableLED m_led{0};
// std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;  // Reuse the buffer
// // Store what the last hue of the first pixel is
// int firstPixelHue = 0;

// void Rainbow() {
//   // For every pixel
//   for (int i = 0; i < kLength; i++) {
//     // Calculate the hue - hue is easier for rainbows because the color
//     // shape is a circle so only one value needs to process
//     const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
//     // Set the value
//     m_ledBuffer[i].SetHSV(pixelHue, 255, 64);
//   }
//   // Increase by to make the rainbow "move"
//   firstPixelHue += 2;
//   // Check bounds
//   firstPixelHue %= 180;
// }

double trueMap(double val, double valHigh, double valLow, double newHigh, double newLow)
{
	double midVal = ((valHigh - valLow) / 2) + valLow;
	double newMidVal = ((newHigh - newLow) / 2) + newLow;
	double ratio = (newHigh - newLow) / (valHigh - valLow);
	return (((val - midVal) * ratio) + newMidVal);
}

void calibratePigeon() {
  pigeon.SetAccumZAngle(0);
  pigeon.SetCompassAngle(0);
  pigeon.SetCompassDeclination(0);
  pigeon.SetFusedHeading(0);
  pigeon.SetFusedHeadingToCompass(0);
  pigeon.SetYaw(0);
  pigeon.SetYawToCompass(0);
  pigeon.EnterCalibrationMode(ctre::phoenix::sensors::PigeonIMU::CalibrationMode::Accelerometer);
}

void Robot::RobotInit() {
  camera0 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  server = frc::CameraServer::GetInstance()->GetServer();
  camera0.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  // compressor.SetClosedLoopControl(true);
  // compressor.Start();
  timer.Reset();
  timer.Start();
  calibratePigeon();
  sensitivity = 0.5;
  // m_led.SetLength(kLength);
  // m_led.SetData(m_ledBuffer);
  // m_led.Start();
}

void Robot::RobotPeriodic() {
  // Rainbow();
  // m_led.SetData(m_ledBuffer);
}

void Robot::AutonomousInit() {
  timer.Reset();
  timer.Start();
  shootTimer.Reset();
  // outtake.Set(1.0);
  ballStorage.Set(frc::DoubleSolenoid::Value::kOff);
  calibratePigeon();
}

void Robot::AutonomousPeriodic() {
  turn = -trueMap(pigeon.GetCompassHeading(), 90, -90, 1.0, -1.0); //set the robot to turn against the strafe
  if(timer.Get() < 0.2) {
    myRobot.ArcadeDrive(timer.Get() * 5, turn);
  }
  else if(timer.Get() < 4) {
    ballStorage.Set(frc::DoubleSolenoid::Value::kForward);
    myRobot.ArcadeDrive(1.0, turn);
  }
  else if(timer.Get() < 5) {
    myRobot.ArcadeDrive(0.5, 0.5 + turn);
  }
  else if(timer.Get() < 6) {
    myRobot.ArcadeDrive(0.5, turn - 0.5);
  }
  else if(timer.Get() < 8) {
    myRobot.ArcadeDrive(0.8, turn);
    pigeon.GetBiasedAccelerometer(accel);
    if (accel[0] == 0 && accel[1] == 0) {
      myRobot.ArcadeDrive(0.0, 0.0);
    }
  }
  // else if(timer.Get() < 9.5) {
  //   myRobot.ArcadeDrive(0.0, turn);
  //   shootTimer.Start();
  //   outtake.Set(-1);
  //   if(shootTimer.Get() < 0.2) {
  //     ballStorage.Set(false);
  //   }
  // }
  // else if(timer.Get() < 11) {
  //   outtake.Set(0);
  //   ballStorage.Set(true);
  //   myRobot.ArcadeDrive(-0.8, turn);
  // }
}

void Robot::TeleopInit() {
  timer.Reset();
  timer.Start();
  shootTimer.Reset();
  turn = 0;
  speed = 0;
  //sensitivity = -two.GetRawAxis(1);
  ballIn.Set(frc::DoubleSolenoid::Value::kOff);//piston1 no go nyoo
  compressor->Start();
  ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Robot::TeleopPeriodic() {
  compressor->SetClosedLoopControl(true);
  //increase sensitivity with the right bumper
  /*
  piston.Set(true); makes the piston go
  piston.Set(false); makes the piston not go
  */
 //In order to go backwards do piston.Set(DoubleSolenoid::Value::kReverse);
 /*if(stick.GetRawButton(3)) {
    piston1.Set(true);
  }
  else{
    piston1.Set(false);
  }
  */

 if(one.GetRawAxis(3)>0.1){
   intake.Set(0.4);
 }else if(one.GetRawButton(1)){
   intake.Set(-1.0);
 }else{
   intake.Set(0.0);
 }


//  if(one.GetRawButton(2)){
//    ballUnstuck.Set(true);
//  }else{
//    ballUnstuck.Set(false);
//  }

 double outtakeSpeed = -1.0;

//  if(one.GetRawAxis(2)>0.1){
//    shootTimer.Start();
//    outtake.Set(outtakeSpeed);
//    if (shootTimer.Get() > .75) {
//      ballStorage.Set(false);
//    }
//  }
//  else if(one.GetRawButton(2)){
//    outtake.Set(-outtakeSpeed);
//    ballStorage.Set(false);
//  }
//  else if (!(one.GetRawAxis(2)>0.1)&&!one.GetRawButton(3)){
//    outtake.Set(0);
//    ballStorage.Set(true);
//    shootTimer.Reset();
//  }
  if (one.GetRawButton(3)) {
    ballStorage.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else if (one.GetRawButton(4)) {
    ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else if (one.GetRawButton(9)) {
    ballStorage.Set(frc::DoubleSolenoid::Value::kOff);
  }
  if(one.GetRawButton(5)) {
    in.Set(1.0);
    out.Set(0.0);
    //ballIn.Set(frc::DoubleSolenoid::Value::kForward);//piston1 go 
  }
  else if (!one.GetRawButton(5)&&one.GetRawButton(6)) {
    in.Set(0.0);
    out.Set(1.0);
    //ballIn.Set(frc::DoubleSolenoid::Value::kReverse);//piston1 go shwoop
  }
  else if (!one.GetRawButton(6)&&!one.GetRawButton(5)){
    in.Set(0.0);
    out.Set(0.0);
    //ballIn.Set(frc::DoubleSolenoid::Value::kOff);//piston1 stop
  }
/*
  if(one.GetRawButton(4)) {
    rightArm.Set(DoubleSolenoid::Value::kForward);//piston1 go nyoom
    leftArm.Set(DoubleSolenoid::Value::kForward);//piston1 go nyoom
  }
  else if (one.GetRawButton(5)) {
    rightArm.Set(DoubleSolenoid::Value::kReverse);//piston1 go shwoop
    leftArm.Set(DoubleSolenoid::Value::kReverse);//piston1 go shwoop
  }
  else{
    rightArm.Set(DoubleSolenoid::Value::kOff);//piston1 stop
    leftArm.Set(DoubleSolenoid::Value::kOff);//piston1 stop
  }
  sensitivity = -two.GetRawAxis(1);
  */
  /*
  if (stick.GetRawButton(9) && sensitivity < 1.0) {
    sensitivity += 0.01;
  }
  else if (stick.GetRawButton(9)) {
    sensitivity += 0;
  } 
  else if (stick.GetRawButton(8) && sensitivity > 0.0) {
    sensitivity -= 0.01;
  }
  else if (stick.GetRawButton(8)) {
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
  
  
    //wheel.Set(0.3);
    //Solenoid.Set
  
  //turn with bumpers, too jittery
  /*if(stick.GetRawButton(7)){
     turn = (-1 * sensitivity);
  } else if(stick.GetRawButton(8)){
     turn = (1 * sensitivity);
  } else{
    turn = 0;
  }
  */
 //sensitivity = one.GetRawAxis(2);
  /*if(one.GetRawAxis(0)>0.2||one.GetRawAxis(0)<-0.2){
			tN=one.GetRawAxis(0);
		}else{
      tN=0;
    }
  if(one.GetRawAxis(1)>0.2||one.GetRawAxis(1)<-0.2){
			sP=one.GetRawAxis(1);
  }else{
    sP=0;
  }
  */
 if (one.GetPOV(0) && sensitivity < 1.0) {
   sensitivity += 0.01;
 }
 else if (one.GetPOV(0) && sensitivity >= 1.0) {
   sensitivity = 1.0;
 }
 else if (one.GetPOV(180) && sensitivity > 0.0) {
   sensitivity -= 0.01;
 }
 else if (one.GetPOV(180) && sensitivity <= 0.0) {
   sensitivity = 0.0;
 }
  speed = one.GetRawAxis(1);
  turn = one.GetRawAxis(4);
  /*
  if (speed >= 0) {
    turn = ((tN * sensitivity)+(speed/4))+0.1;
  }
  else {
    turn = ((tN*sensitivity)-(speed/4))+0.15;
  }
  */
  myRobot.ArcadeDrive(speed*sensitivity, turn*sensitivity);

  pan.Set(trueMap(two.GetRawAxis(0),1,-1,1,0));
  tilt.Set(trueMap(two.GetRawAxis(1),-1,1,1,0));
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif