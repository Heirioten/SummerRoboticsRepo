// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.auto.AprilTagAlignAmp;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.PIDAlign;
import frc.robot.commands.auto.SwerveTrajectoryCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystemIOSparkMax;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController
    = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private TeleopCommand teleopCommand;

  private SendableChooser<Integer> driveChooser = new SendableChooser<Integer>();
  private SendableChooser<Integer> tempAutoChooser = new SendableChooser<Integer>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());
    visionSubsystem = new VisionSubsystem();

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

    // driverController.a().whileTrue(
    //   new SwerveTrajectoryCommand(
    //     driveSubsystem,
    //     new Pose2d(1.7, 7.6, Rotation2d.fromDegrees(90)),
    //     new PathConstraints(3, 3, 6.28, 6.28),
    //     0.0,
    //     0//,
    //     // "amp"
    //   )
    // );

    driverController.a().whileTrue(
      new PIDAlign(
        driveSubsystem,
        new Pose2d(1.7, 7.6, Rotation2d.fromDegrees(90)), RobotContainer::isRed,
        10.0, 0, 0.0,
        10.0, 0, 0.0
      )
    );

    // driverController.a().whileTrue(
      // new AprilTagAlignAmp(driveSubsystem, visionSubsystem, getDriveConfig(), getDriveConfig(), getDriveConfig(), getDriveConfig(), getDriveConfig(), getDriveConfig(), getDriveConfig())
    // )


  }

  public Command getAutonomousCommand() {
    return Autos.B1R3_2Cube();
  }

  public int getDriveConfig() {
    return (int) driveChooser.getSelected();
  }

  /**
   * Defaults to blue
   * @return Alliance
   */
  public static Alliance getAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if(alliance.isPresent()) {
      return alliance.get();
    }
    else return Alliance.Blue;
  }

  public static boolean isRed() {
    return getAlliance() == Alliance.Red;
  }
}
