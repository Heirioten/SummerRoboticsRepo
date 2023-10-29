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

  // our operator joystick
  private final CommandJoystick driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);

  // all commands and subsystems get set up here
  private GripperSubsystem gripperSubsystem = new GripperSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private ExtensionArmSubsystem extensionArmSubsystem = new ExtensionArmSubsystem();
  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, this);


  // spianris wanted us to be able to swtich between arcade and tank from dashboard
  private SendableChooser<Integer> driveChooser = new SendableChooser<Integer>();

  // this is to swtic between auto doing nothing and auto doing who knows what
  private SendableChooser<Integer> tempAutoChooser = new SendableChooser<Integer>();

  // so we can set up voltage constraints and max speed and stuffs
  private TrajectoryConfig trajectoryConfig;
  // limit to 6v because if i hqad to guess its more precise at lower speeds
  private DifferentialDriveVoltageConstraint constraint;


  public RobotContainer()
  {
    // set up buttons and joysticks aand such
    configureBindings();

    driveChooser.addOption("Tank", 1);
    driveChooser.setDefaultOption("Arcade", 0);
    driveChooser.addOption("SimF310", 2); // the only time i ever use an f310 is at home because i dont own an xbox controller. we have an accessor method getDriveOption or something in this file that we access from inside TeleopCommand

    tempAutoChooser.setDefaultOption("Nothing", 0);
    tempAutoChooser.setDefaultOption("Test Trajectory Auto", 1);

    // https://en.wikipedia.org/wiki/Motor_constants you dont need a deep understanding of this, just know that motor constants help us translate between desired speed and how many volts to output to the controller
    constraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(OperatorConstants.kS, OperatorConstants.kV),
      OperatorConstants.kinematics,
      6); // 6 volts MAX
    trajectoryConfig = new TrajectoryConfig(OperatorConstants.kMaxAutoVel, OperatorConstants.kMaxAutoAccel) // i dont wanna go too fast!!
    .setKinematics(OperatorConstants.kinematics).addConstraint(constraint); // kinematics helps it know how to turn (distance between the wheels is important)
  }

  private void configureBindings() 
  {

    // outputs selectors to 
    SmartDashboard.putData("DriveChooser", driveChooser);
    SmartDashboard.putData("TempAutoChooser", tempAutoChooser);


    // https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html#trigger-bindings

    // Controller 0 Button 4 opens gripper
    driverController.button(4).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.OPEN); } )); // lambda function https://www.w3schools.com/java/java_lambda.asp
    
    // Controller 0 Button 5 closes gripper
    driverController.button(5).onTrue(Commands.runOnce(
    () -> { gripperSubsystem.setState(GripperState.CLOSE); } ));

    // Manual control of pivot only when Controller 0 Axis 1 exceeds deadzone in positive or negative direction 
    driverController.axisGreaterThan(1, OperatorConstants.kDeadzone).whileTrue(
      Commands.run(() -> { // Commands.run -- run indefinitely. the whileTrue (right above) will stop the command once the trigger (axisGreaterThan) returns a false
         pivotSubsystem.adjustSetpoint(-driverController.getRawAxis(1) * OperatorConstants.kPivotSpeed); }, // using this constant to adjust the rate of change. the adjustSetpoint function just adds whatever we give it to the setpoint of the pivot
      pivotSubsystem));

    driverController.axisLessThan(1, -OperatorConstants.kDeadzone).whileTrue(
      Commands.run(() -> { pivotSubsystem.adjustSetpoint(-driverController.getRawAxis(1) * OperatorConstants.kPivotSpeed); },
      pivotSubsystem));

    // Manual control of extension only when Controller 0 Axis 0 exceeds deadzone in positive or negative direction (Non-PID loop)
    driverController.axisGreaterThan(0, OperatorConstants.kExtensionDeadzone).whileTrue( // c.f. pivot
      Commands.run(() -> { extensionArmSubsystem.adjustSetpoint(
        driverController.getRawAxis(0) * OperatorConstants.kExtensionSpeed_OUT); },
      extensionArmSubsystem));

    driverController.axisLessThan(0, -OperatorConstants.kExtensionDeadzone).whileTrue( // c.f. pivot
      Commands.run(() -> { extensionArmSubsystem.adjustSetpoint(
        driverController.getRawAxis(0) * OperatorConstants.kExtensionSpeed_IN); },
      extensionArmSubsystem));

    // Look around at GripperSubsystem, PivotSubsystem, and ExtensionArmSubsystem now
    
    driveSubsystem.setDefaultCommand(teleopCommand); // default command -- the command to run when no other commands want to run. when any other command is called, this one stops, and when said other command ends, this will resume.
  }

  public Command getAutonomousCommand() // This is called in autonomousInit and the return value (type Command) is then executed
  {
    if(tempAutoChooser.getSelected() == 0) return Commands.print("No autonomous command configured");
    else {
      return getRamseteCommand(
        TrajectoryGenerator.generateTrajectory(new Pose2d() /* new Pose2d() means x=0 y=0 and theta=0 */, List.of(
          new Translation2d(2, 1), // x=2 meters y=1 meter
          new Translation2d(4, 5) // as above
        ),
        new Pose2d(10, 2, /* x=10 y=2 */ Rotation2d.fromDegrees(0)), trajectoryConfig)
      );
    }
  }

  public Command getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand( // This has a few different constructors, but this one is most straightforward imo
      trajectory,
      driveSubsystem::getPose, // Current position and angle of the robot
      new RamseteController(OperatorConstants.kB, OperatorConstants.kZeta), // these two variables basically control how aggressive/passive the path follower is
      new SimpleMotorFeedforward(OperatorConstants.kS, OperatorConstants.kV), // helps convert desired velocity to voltage that we should output
      OperatorConstants.kinematics, // tells the controller how aggressively to turn (kinematics calculates left-right velocity difference to turn given the distance between wheels)
      driveSubsystem::getWheelSpeeds, // straightforward
      new PIDController(OperatorConstants.kP, 0, 0), // see note on PID control in Robot.java
      new PIDController(OperatorConstants.kP, 0, 0), // ^
      driveSubsystem::tankDriveVolts, // function to set left side voltage and right side voltage
      driveSubsystem // it wants a copy of our drivesubsystem for some reason??
    );
  }

  public int getDriveConfig() { // accesor method
    return (int) driveChooser.getSelected(); // this gets selected by clicking on the choice you want in smartdashboard
  }
}
