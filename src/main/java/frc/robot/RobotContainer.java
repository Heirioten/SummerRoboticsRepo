// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private RunCommand pivotControl;
  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings() 
  {
    driverController.button(4).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.OPEN); } ));
    
    driverController.button(5).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.CLOSE); } ));

    // Commands.run(() -> {pivotSubsystem.adjustSetpoint(driverController.getRawAxis(1));});
    pivotControl = new RunCommand(
      () -> { pivotSubsystem.adjustSetpoint(-driverController.getRawAxis(1)); },
    pivotSubsystem);
    pivotSubsystem.setDefaultCommand(pivotControl);

  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
