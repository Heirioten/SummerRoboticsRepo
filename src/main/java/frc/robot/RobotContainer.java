// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystemIOSparkMax;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer
{
  // private final CommandJoystick driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private DriveSubsystem driveSubsystem;
  private TeleopCommand teleopCommand;

  private SendableChooser<Integer> driveChooser = new SendableChooser<Integer>();
  private SendableChooser<Integer> tempAutoChooser = new SendableChooser<Integer>();

  private TrajectoryConfig trajectoryConfig;
  private DifferentialDriveVoltageConstraint constraint;


  public RobotContainer()
  {
    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());
    teleopCommand = new TeleopCommand(driveSubsystem, this);

    driveChooser.setDefaultOption("Arcade", 0);

    tempAutoChooser.setDefaultOption("Nothing", 0);
    tempAutoChooser.setDefaultOption("Test Trajectory Auto", 1);

    constraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(OperatorConstants.kS, OperatorConstants.kV),
      OperatorConstants.kinematics,
      6);
    trajectoryConfig = new TrajectoryConfig(0.5, 0.2)
    .setKinematics(OperatorConstants.kinematics).addConstraint(constraint);

    configureBindings();
  }

  private void configureBindings() 
  {
    // SmartDashboard.putData("DriveChooser", driveChooser);
    driveSubsystem.setDefaultCommand(teleopCommand);
  }

  public Command getAutonomousCommand() 
  {
      driveSubsystem.setPose(0, 0, 0);
      return getRamseteCommand(
        TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(
          new Translation2d(2.1, -0.25),
          new Translation2d(2.3, -2),
          new Translation2d(2.3, -4.5),
          new Translation2d(7.2, -4.5)
        ),
        new Pose2d(7.2, 0.5 , Rotation2d.fromDegrees(0)), trajectoryConfig)
      );

  }

  public Command getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory,
      driveSubsystem::getPose,
      new RamseteController(OperatorConstants.kB, OperatorConstants.kZeta),
      new SimpleMotorFeedforward(OperatorConstants.kS, OperatorConstants.kV),
      OperatorConstants.kinematics,
      driveSubsystem::getWheelSpeeds,
      new PIDController(OperatorConstants.kP, 0, 0),
      new PIDController(OperatorConstants.kP, 0, 0),
      driveSubsystem::tankDriveVolts,
      driveSubsystem
    );
  }

  public int getDriveConfig() {
    return (int) driveChooser.getSelected();
  }
}
