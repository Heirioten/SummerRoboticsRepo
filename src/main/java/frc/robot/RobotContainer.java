// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.GripperSubsystem.GripperState;

public class RobotContainer
{
  private final CommandJoystick driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private GripperSubsystem gripperSubsystem = new GripperSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings() 
  {
    // Controller 0 Button 4 opens gripper
    driverController.button(4).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.OPEN); } ));
    
    // Controller 0 Button 5 closes gripper
    driverController.button(5).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.CLOSE); } ));


    // Manual control of pivot only when Controller 0 Axis 1 exceeds deadzone in positive or negative direction 
    driverController.axisGreaterThan(1, OperatorConstants.kPivotDeadzone).whileTrue(
      Commands.run(() -> { pivotSubsystem.adjustSetpoint(-driverController.getRawAxis(1)); },
      pivotSubsystem));

    driverController.axisLessThan(1, -OperatorConstants.kPivotDeadzone).whileTrue(
      Commands.run(() -> { pivotSubsystem.adjustSetpoint(-driverController.getRawAxis(1)); },
      pivotSubsystem));

  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
