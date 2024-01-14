// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends Command {

  DriveSubsystem driveSubsystem;
  XboxController controller;
  SlewRateLimiter filterX;
  SlewRateLimiter filterY;
  RobotContainer container;

  public TeleopCommand(DriveSubsystem driveSubsystem, RobotContainer container) {
    this.driveSubsystem = driveSubsystem;
    this.container = container;
    controller = new XboxController(0);
    addRequirements(driveSubsystem);
    filterX = new SlewRateLimiter(OperatorConstants.kDriveRateLimit);
    filterY = new SlewRateLimiter(OperatorConstants.kDriveRateLimit);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double filteredX = 0.0; // Forward/backward
    if (!Robot.isReal()) filteredX = filterX.calculate(-controller.getRawAxis(1)); // Simulated axis
    else filteredX = filterX.calculate(-controller.getLeftY()); // Real axis
    if (Math.abs(filteredX) < 0.15) filteredX = 0; // Deadzone

    double filteredY = 0.0;
    if (!Robot.isReal()) filteredY = filterY.calculate(-controller.getRawAxis(0));
    else filteredY = filterY.calculate(-controller.getLeftX());
    if (Math.abs(filteredY) < 0.15) filteredY = 0;

    double omega = 0.0; // Turning
    if (!Robot.isReal()) omega = -controller.getRawAxis(2);
    else omega = -controller.getRightX();
    if (Math.abs(omega) < 0.15) omega = 0;

    driveSubsystem.arcadeDriveFieldOriented(filteredX, filteredY, omega);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
