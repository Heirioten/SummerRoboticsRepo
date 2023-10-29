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
 
  DriveSubsystem driveSubsystem;
  XboxController controller;
  SlewRateLimiter filter;
  RobotContainer container;
  
  public TeleopCommand(DriveSubsystem driveSubsystem, RobotContainer container) 
  {
    this.driveSubsystem = driveSubsystem;
    this.container = container;
    controller = new XboxController(1);
    addRequirements(driveSubsystem);
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
    driveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
