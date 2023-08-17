// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {
 
  DriveSubsystem driveSubsystem;
  XboxController controller;
  
  /** Creates a new TeleopCommand. */
  public TeleopCommand(DriveSubsystem driveSubsystem) 
  {
    this.driveSubsystem = driveSubsystem;
    controller = new XboxController(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    driveSubsystem.arcadeDrive(controller.getLeftY(), controller.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    driveSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
