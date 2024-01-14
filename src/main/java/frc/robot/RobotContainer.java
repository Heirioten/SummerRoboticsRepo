// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystemIOSparkMax;

public class RobotContainer {
  // private final CommandJoystick driverController = new
  // CommandJoystick(OperatorConstants.kDriverControllerPort);
  private DriveSubsystem driveSubsystem;
  private TeleopCommand teleopCommand;

  private SendableChooser<Integer> driveChooser = new SendableChooser<Integer>();
  private SendableChooser<Integer> tempAutoChooser = new SendableChooser<Integer>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());

    Autos.constructAutoBuilder(driveSubsystem);

    teleopCommand = new TeleopCommand(driveSubsystem, this);

    driveChooser.setDefaultOption("Arcade", 0);

    tempAutoChooser.setDefaultOption("Nothing", 0);
    tempAutoChooser.setDefaultOption("Test Trajectory Auto", 1);

    configureBindings();
  }

  private void configureBindings() {
    // SmartDashboard.putData("DriveChooser", driveChooser);
    driveSubsystem.setDefaultCommand(teleopCommand);
  }

  public Command getAutonomousCommand() {
    return Autos.B1R3_2Cube();
  }

  public int getDriveConfig() {
    return (int) driveChooser.getSelected();
  }
}
