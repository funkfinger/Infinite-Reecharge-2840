#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

class RobotContainer {
  public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
  private:
    frc::Joystick m_driverController{0};
    DriveSubsystem m_drive;

    frc2::InstantCommand m_driveHalfSpeed{[this] { m_drive.SetMaxOutput(0.5); },
                                        {}};
    frc2::InstantCommand m_driveFullSpeed{[this] { m_drive.SetMaxOutput(1); },
                                        {}};

  // The chooser for the autonomous routines
    frc::SendableChooser<frc2::Command*> m_chooser;

    void ConfigureButtonBindings();
};