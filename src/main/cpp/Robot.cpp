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
//#include <frc/WPILib.h>
#include <frc/PowerDistributionPanel.h>
#include <cameraServer/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/Servo.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
// #include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
//#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
// #include <C:/Users/Programming/.gradle/caches/transforms-2/files-2.1/bcaf719eab4760b22c0d3083c34a9489/hal-cpp-2020.3.2-headers/mockdata/MockHooks.h>
// #include <C:/Users/Programming/.gradle/caches/transforms-2/files-2.1/bcaf719eab4760b22c0d3083c34a9489/hal-cpp-2020.3.2-headers/mockdata/SimDeviceData.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <frc/RobotDrive.h>

#include "rev/SparkMax.h"
#include <frc/Compressor.h>
#include <frc/Talon.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
// #include <frc/AddressableLED.h>
#include <math.h>

cs::UsbCamera camera0;
cs::UsbCamera camera1;
cs::VideoSink server;
frc::Joystick one{0}, two{1};
//rev::SparkMax intake{4}, outtake{5};
rev::SparkMax top{5}, intake{4};
frc::Servo pan{6},tilt{7};
int stage = 0;
double xyz[] = {0.0, 0.0, 0.0};
// frc::Talon frontLeft{2}, frontRight{0}, backRight{3}, backLeft{1}, out{8};

ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontLeft = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *backLeft= new ctre::phoenix::motorcontrol::can::WPI_TalonFX(3);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *backRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
ctre::phoenix::motorcontrol::can::TalonSRX *bottom = new ctre::phoenix::motorcontrol::can::TalonSRX(10);

//ctre::phoenix::motorcontrol::can::WPI_TalonFX::WPI_TalonFX * frontRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX::WPI_TalonFX(1);
// WPI_TalonFX * _rghtFollower = new WPI_TalonFX(3);
// WPI_TalonFX * _leftFront = new WPI_TalonFX(2);
// WPI_TalonFX * _leftFollower = new WPI_TalonFX(4);
//ctre::phoenix::motorcontrol::can::VictorSPX frontLeft{2}, frontRight{1}, backLeft{3}, backRight{0}, panel{10};
frc::RobotDrive myRobot{*frontLeft, *backLeft, *frontRight, *backRight};
frc::Timer timer, shootTimer;


//frc::SendableChooser autoChoice;
frc::Solenoid ballUnstuck{0};
frc::DoubleSolenoid ballIn{1, 2}, ballStorage{3, 7};
frc::Compressor *compressor = new frc::Compressor(0);

ctre::phoenix::sensors::PigeonIMU pigeon{bottom};
//0.65 is the ideal sensitivity
double speed = 0.0, turn = 0.0, sensitivity = 1.0, turnKey, avgDist = 0.0, currentTime = 0.0, prevTime = 0.0, maxTime = 0, maxSpeed = 0;
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

void resetEncoders() {
  frontLeft->SetSelectedSensorPosition(0);
  backLeft->SetSelectedSensorPosition(0);
  frontRight->SetSelectedSensorPosition(0);
  backRight->SetSelectedSensorPosition(0);
}

double trueMap(double val, double valHigh, double valLow, double newHigh, double newLow)
{
	double midVal = ((valHigh - valLow) / 2) + valLow;
	double newMidVal = ((newHigh - newLow) / 2) + newLow;
	double ratio = (newHigh - newLow) / (valHigh - valLow);
	return (((val - midVal) * ratio) + newMidVal);
}

void calibratePigeon() {
  pigeon.SetAccumZAngle(0);
  pigeon.SetCompassAngle(0.0);
  pigeon.SetCompassDeclination(0);
  pigeon.SetFusedHeading(0);
  pigeon.SetFusedHeadingToCompass(0);
  pigeon.SetYaw(0);
  pigeon.SetYawToCompass(0);
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
  // m_led.SetLength(kLength);
  // m_led.SetData(m_ledBuffer);
  // m_led.Start();
  frontLeft->SetSelectedSensorPosition(0.0);
  backLeft->SetSelectedSensorPosition(0.0);
  frontRight->SetSelectedSensorPosition(0.0);
  backRight->SetSelectedSensorPosition(0.0);
}

void Robot::RobotPeriodic() {
  pigeon.GetAccumGyro(xyz);
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  frc::SmartDashboard::PutNumber("FrontLeft Distance: ", (double)frontLeft->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("FrontRight Distance: ", (double)frontRight->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("Backleft Distance: ", (double)backLeft->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("Backright Distance: ", (double)backRight->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("FrontLeft Velocity: ", (double)frontLeft->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("FrontRight Velocity: ", (double)frontRight->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("Backleft Velocity: ", (double)backLeft->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("Backright Velocity: ", (double)backRight->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("Heading: ",pigeon.GetAbsoluteCompassHeading());
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
  currentTime = 0;
  prevTime = 0;
  stage = 0;
}

void Robot::AutonomousPeriodic() {
  avgDist = ((double)frontLeft->GetSelectedSensorPosition()*-1+(double)backLeft->GetSelectedSensorPosition()*-1+(double)frontRight->GetSelectedSensorPosition()+(double)backRight->GetSelectedSensorPosition())/4.0;
  avgDist /= 6612.5;
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  frc::SmartDashboard::PutNumber("Average Distance", avgDist);
  frc::SmartDashboard::PutNumber("Stage Time: ", currentTime-prevTime);
  frc::SmartDashboard::PutNumber("Stage: ", stage+1);
  currentTime = timer.Get();
  myRobot.ArcadeDrive(speed, turn);
  if (stage == 0) {//arrives at position 2
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 10.0) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 1) {
    myRobot.ArcadeDrive(0.0, 0.7);
    if ((int)xyz[2] % 360 >= 90.0) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 2) {//arrives at position 3
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 4.75) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 3) {
    myRobot.ArcadeDrive(0.0, 0.7);
    if ((int)xyz[2] % 360 >= 180) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 4) {//arrives at position 4
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 2.5) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 5) {
    myRobot.ArcadeDrive(0.0, 0.7);
    if ((int)xyz[2] % 360 >= 270) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 6) {//arrives at position 5
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 2.5) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 7) {
    myRobot.ArcadeDrive(0.0, 0.7);
    if ((int)xyz[2] % 360 <= 10) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 8) {//arrives at position 6
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 9.75) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 9) {
    myRobot.ArcadeDrive(0.0, -0.7);
    if ((int)xyz[2] % 360 <= 270 && (int)xyz[2] % 360 >= 90) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 10) {//arrives at position 7
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 4.25) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 11) {
    myRobot.ArcadeDrive(0.0, -0.7);
    if ((int)xyz[2] % 360 <= 180) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 12) {//arrives at position 8
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 2.5) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 13) {
    myRobot.ArcadeDrive(0.0, -0.7);
    if ((int)xyz[2] % 360 <= 90) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 14) {//arrives at position 9
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 7.25) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 15) {
    myRobot.ArcadeDrive(0.0, -0.7);
    if ((int)xyz[2] % 360 >= 270) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 16) {//arrives at position 10
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 7.5) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 17) {
    myRobot.ArcadeDrive(0.0, -0.7);
    if ((int)xyz[2] % 360 >= 270) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 18) {//arrives at position 11
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist >= 2.5) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 19) {
    myRobot.ArcadeDrive(0.0, -0.7);
    if ((int)xyz[2] % 360 <= 180) {stage++; prevTime = currentTime; resetEncoders();}
  }
  else if (stage == 20) {
    myRobot.ArcadeDrive(1.0, 0.0);
    if (avgDist > 30) {stage++; prevTime = currentTime; resetEncoders();}
  }


  // turn = -trueMap(pigeon.GetAbsoluteCompassHeading()-180, 180, -180, 1.0, -1.0); //set the robot to turn against the strafe
  // if(timer.Get() < 0.2) {
  //   myRobot.ArcadeDrive(timer.Get() * 5, turn);
  // }
  // else if(timer.Get() < 4) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kForward);
  //   myRobot.ArcadeDrive(1.0, turn);
  // }
  // else if(timer.Get() < 5) {
  //   myRobot.ArcadeDrive(0.5, 0.5 + turn);
  // }
  // else if(timer.Get() < 6) {
  //   myRobot.ArcadeDrive(0.5, turn - 0.5);
  // }
  // else if(timer.Get() < 8) {
  //   myRobot.ArcadeDrive(0.8, turn);
  //   pigeon.GetBiasedAccelerometer(accel);
  //   if (accel[0] == 0 && accel[1] == 0) {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //   }
  // }
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
  frontLeft->SetSelectedSensorPosition(0.0);
  backLeft->SetSelectedSensorPosition(0.0);
  frontRight->SetSelectedSensorPosition(0.0);
  backRight->SetSelectedSensorPosition(0.0);
  timer.Reset();
  timer.Start();
  shootTimer.Reset();
  turn = 0.0;
  speed = 0.0;
  //sensitivity = -two.GetRawAxis(1);
  ballIn.Set(frc::DoubleSolenoid::Value::kOff);//piston1 no go nyoo
  compressor->Start();
  ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
  calibratePigeon();
}

void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("Heading x: ", xyz[0]);
  frc::SmartDashboard::PutNumber("Heading y: ", xyz[1]);
  frc::SmartDashboard::PutNumber("Heading z: ", xyz[2]);
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
  // if (one.GetRawButton(3)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kForward);
  // }
  // else if (one.GetRawButton(4)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
  // }
  // else if (one.GetRawButton(9)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kOff);
  // }

  if (one.GetRawButton(2)) {
    top.Set(-0.9);
    bottom->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.85);
  }
  else if (!one.GetRawButton(2)&&one.GetRawButton(3)) {
    top.Set(-0.68);
    bottom->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.63);
  }
  else if (!one.GetRawButton(2)&&!one.GetRawButton(3)&&one.GetRawButton(4)) {
    top.Set(-1.0);
    bottom->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1.0);
  }
  else if (!one.GetRawButton(2) && !one.GetRawButton(3)&&!one.GetRawButton(4)) {
    top.Set(0.0);
    bottom->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
  }
  
  // if(one.GetRawButton(5)) {
  //   in.Set(1.0);
  //   out->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.0);
  //   //ballIn.Set(frc::DoubleSolenoid::Value::kForward);//piston1 go 
  // }
  // else if (!one.GetRawButton(5)&&one.GetRawButton(6)) {
  //   in.Set(0.0);
  //   out->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1.0);
  //   //ballIn.Set(frc::DoubleSolenoid::Value::kReverse);//piston1 go shwoop
  // }
  // else if (!one.GetRawButton(6)&&!one.GetRawButton(5)){
  //   in.Set(0.0);
  //   out->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
  //   //ballIn.Set(frc::DoubleSolenoid::Value::kOff);//piston1 stop
  // }
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
//  if (one.GetPOV(0) && sensitivity < 1.0) {
//    sensitivity += 0.01;
//  }
//  else if (one.GetPOV(0) && sensitivity >= 1.0) {
//    sensitivity = 1.0;
//  }
//  else if (one.GetPOV(180) && sensitivity > 0.0) {
//    sensitivity -= 0.01;
//  }
//  else if (one.GetPOV(180) && sensitivity <= 0.0) {
//    sensitivity = 0.0;
//  }
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

void Robot::TestPeriodic() {
  // avgDist = ((double)frontLeft->GetSelectedSensorPosition()+(double)backLeft->GetSelectedSensorPosition()+(double)frontRight->GetSelectedSensorPosition()+(double)backRight->GetSelectedSensorPosition())/4.0;
  // frc::SmartDashboard::PutNumber("Timer", timer.Get());
  // frc::SmartDashboard::PutNumber("Average Distance", avgDist);
  // frc::SmartDashboard::PutNumber("Stage Time: ", currentTime-prevTime);
  // currentTime = timer.Get();
  // if (stage % 3 == 0) {
  //   myRobot.ArcadeDrive(0.2, 0.0);
  //   if (avgDist >= 5.0) {stage++; prevTime = currentTime;}
  // }
  // else if (stage % 3 == 1) {
  //   if (currentTime-prevTime >= 2.0) {stage++; prevTime = currentTime;}
  // }
  // else if (stage % 3 == 2) {
  //   myRobot.ArcadeDrive(-0.2, 0.0);
  //   if (avgDist <= 0.0) {stage = 0; prevTime = currentTime;}
  // }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif