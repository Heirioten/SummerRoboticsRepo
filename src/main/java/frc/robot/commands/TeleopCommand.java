// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {

  // this is our loop that actually controls the drivetrain of the robot
 
  DriveSubsystem driveSubsystem;
  XboxController controller;
  SlewRateLimiter filter; // slows our (de/ac)celeration so that we dont tip over from a sudden jerk
  RobotContainer container; // we want this reference so that we can access the drive config (remember spinaris wanted to switch between arcade and tank drive on the fly if desired)
  
  public TeleopCommand(DriveSubsystem driveSubsystem, RobotContainer container) 
  {
    this.driveSubsystem = driveSubsystem;
    this.container = container;
    controller = new XboxController(1);
    addRequirements(driveSubsystem); // commands should tell the CommandScheduler the subsystems they require, so that the scheduler will know which commands can and cannot run concurrently
    filter = new SlewRateLimiter(OperatorConstants.kDriveRateLimit);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    if(container.getDriveConfig() == 0)
      driveSubsystem.arcadeDrive(filter.calculate(controller.getLeftY() / OperatorConstants.kDriveSpeedDivisor), (controller.getRightX() / OperatorConstants.kDriveTurnDivisor));
    else if(container.getDriveConfig() == 1) {
      driveSubsystem.tankDrive(controller.getLeftY(), controller.getRightY());
    } else {
      driveSubsystem.arcadeDrive(filter.calculate(-controller.getRawAxis(1)), 
      controller.getRawAxis(2));
    }

   }

  @Override
  public void end(boolean interrupted) 
  {
    // if we dont zero everything on command end, if we call another command using the DriveSubsystem that doenst itself zero everything immedaitely, we'll keep going with whatever the driver was doing at the time
    // (there are actually no other commands using the DriveSubsystem at the moment, but build code that is adatpable or something)
    driveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false; // so relatable
  }
}

// now read DriveSubsystem. this one sucks a bit (a lot)