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
 

public class PivotSubsystem extends PIDSubsystem {

  CANSparkMax pivot;
  RelativeEncoder encoder;

  public PivotSubsystem() 
  {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.25, 0, 0.01));
    
    pivot = new CANSparkMax(5, MotorType.kBrushless);
    encoder = pivot.getEncoder();

    pivot.restoreFactoryDefaults();

    enable();
  }

  @Override
  public void useOutput(double output, double setpoint) 
  {
    pivot.set(output);
  }

  @Override
  public double getMeasurement() 
  {
    return encoder.getPosition();
  }

  @Override
  public void periodic()
  {
    super.periodic();
    SmartDashboard.putBoolean("disabled", Robot.isRobotDisabled());
    if(Robot.isRobotDisabled())
    {
      setSetpoint(encoder.getPosition());
    }
    SmartDashboard.putNumber("Pivot Encoder", encoder.getPosition());
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
  public double adjustSetpoint(double delta)
  {
    double setpoint = getSetpoint();
    setpoint += (delta * 0.25);
    setSetpoint(setpoint);
    return setpoint;
  }
}
