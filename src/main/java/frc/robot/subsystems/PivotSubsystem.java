// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 

public class PivotSubsystem extends PIDSubsystem { // special type of subsystem with functions for PID built in
  // we need PID control on the pivot motor because gravity will otherwise pull the arm down

  CANSparkMax pivot;
  RelativeEncoder encoder;

  public PivotSubsystem() 
  {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.25, 0, 0.01)); // i dont know why we have a D on this. i think we were just bored of typing too many zeroes
    
    pivot = new CANSparkMax(5, MotorType.kBrushless);
    encoder = pivot.getEncoder();

    pivot.restoreFactoryDefaults();

    enable(); // this enables the PID controller for the subsystem (line 17)
  }

  @Override
  public void useOutput(double output, double setpoint) // subsystem PID needs to know how to set the motor. the output variable is the calculated PID output. we dont really need the setpoint for our case
  {
    pivot.set(output);
  }

  @Override
  public double getMeasurement() // subsystem PID needs to know what we're measuring
  {
    return encoder.getPosition();
  }

  @Override
  public void periodic()
  {
    super.periodic();
    SmartDashboard.putBoolean("disabled", Robot.isRobotDisabled());
    if(Robot.isRobotDisabled()) // see Robot.java:50-51
    {
      setSetpoint(encoder.getPosition()); // see Robot.java:50-51
    }
    SmartDashboard.putNumber("Pivot Encoder", encoder.getPosition()); // telemetry
    SmartDashboard.putNumber("Pivot Setpoint", getSetpoint());
    SmartDashboard.putNumber("Pivot Value", pivot.get());
  }

  /**
   * @param val Value from [-1f, 1f] to feed motor
   */
  public void setMotor(double val) {
    pivot.set(val);
  }

  /**
   * @param delta The change desired in setpoint
   * @return The new setpoint after adding the delta
   */
  public double adjustSetpoint(double delta) // finally, the subsystem PID needs to know where we want to know (this is called in RobotContainer using joystick values)
  {
    double setpoint = getSetpoint();
    setpoint += (delta * 0.25);
    setSetpoint(setpoint);
    return setpoint;
  }
}
