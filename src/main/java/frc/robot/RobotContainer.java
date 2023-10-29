// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtensionArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.GripperSubsystem.GripperState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer
{
  private final CommandJoystick driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private GripperSubsystem gripperSubsystem = new GripperSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private ExtensionArmSubsystem extensionArmSubsystem = new ExtensionArmSubsystem();
  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, this);

  private SendableChooser<Integer> driveChooser = new SendableChooser<Integer>();

  private TrajectoryConfig trajectoryConfig;
  private DifferentialDriveVoltageConstraint constraint;


  public RobotContainer()
  {
    configureBindings();

    driveChooser.addOption("Tank", 1);
    driveChooser.setDefaultOption("Arcade", 0);
    driveChooser.addOption("SimF310", 2);

    constraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(OperatorConstants.kS, OperatorConstants.kV),
      OperatorConstants.kinematics,
      6);
    trajectoryConfig = new TrajectoryConfig(OperatorConstants.kMaxAutoVel, OperatorConstants.kMaxAutoAccel)
    .setKinematics(OperatorConstants.kinematics).addConstraint(constraint);
  }

  private void configureBindings() 
  {

    SmartDashboard.putData("DriveChooser", driveChooser);

    // Controller 0 Button 4 opens gripper
    driverController.button(4).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.OPEN); } ));
    
    // Controller 0 Button 5 closes gripper
    driverController.button(5).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.CLOSE); } ));

    // Manual control of pivot only when Controller 0 Axis 1 exceeds deadzone in positive or negative direction 
    driverController.axisGreaterThan(1, OperatorConstants.kDeadzone).whileTrue(
      Commands.run(() -> { pivotSubsystem.adjustSetpoint(-driverController.getRawAxis(1) * OperatorConstants.kPivotSpeed); },
      pivotSubsystem));

    driverController.axisLessThan(1, -OperatorConstants.kDeadzone).whileTrue(
      Commands.run(() -> { pivotSubsystem.adjustSetpoint(-driverController.getRawAxis(1) * OperatorConstants.kPivotSpeed); },
      pivotSubsystem));

    // Manual control of extension only when Controller 0 Axis 0 exceeds deadzone in positive or negative direction (Non-PID loop)
    driverController.axisGreaterThan(0, OperatorConstants.kExtensionDeadzone).whileTrue(
      Commands.run(() -> { extensionArmSubsystem.adjustSetpoint(
        driverController.getRawAxis(0) * OperatorConstants.kExtensionSpeed_OUT); },
      extensionArmSubsystem));

    driverController.axisLessThan(0, -OperatorConstants.kExtensionDeadzone).whileTrue(
      Commands.run(() -> { extensionArmSubsystem.adjustSetpoint(
        driverController.getRawAxis(0) * OperatorConstants.kExtensionSpeed_IN); },
      extensionArmSubsystem));
    
    driveSubsystem.setDefaultCommand(teleopCommand);
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
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
