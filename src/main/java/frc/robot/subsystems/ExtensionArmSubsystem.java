// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class ExtensionArmSubsystem extends PIDSubsystem {
  
  CANSparkMax extension;
  RelativeEncoder encoder;
  AbsoluteEncoder absEncoder;
  
  /** Creates a new ExtensionArmSubsystem. */
  public ExtensionArmSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.1, 0, 0));
    
    extension = new CANSparkMax(6, MotorType.kBrushless);
    encoder = extension.getEncoder();
    absEncoder = extension.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    extension.restoreFactoryDefaults();
    extension.enableSoftLimit(SoftLimitDirection.kForward, true);
    extension.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extension.setSoftLimit(SoftLimitDirection.kForward, 34);
    extension.setSoftLimit(SoftLimitDirection.kReverse, -4);
    extension.setSmartCurrentLimit(50);
    


    enable();
  }

  @Override
  public void useOutput(double output, double setpoint) 
  {
    extension.set(output);
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
    if(Robot.isRobotDisabled())
    {
      setSetpoint(encoder.getPosition());
    }


    SmartDashboard.putNumber("Extension Setpoint", getSetpoint());
    SmartDashboard.putNumber("Extension Motor Voltage", extension.get());
    SmartDashboard.putNumber("Extension Encoder", encoder.getPosition());

    SmartDashboard.putNumber("Spool Encoder Position", encoder.getPosition());
    SmartDashboard.putNumber("Spool Absolute Encoder Position", absEncoder.getPosition());
  }

    /**
   * @param val Value from [-1f, 1f] to feed motor
   */
  public void setMotor(double val) {
    extension.set(val);
  }

  /**
   * @param delta The change desired in setpoint
   * @return The new setpoint after adding the delta
   */
  public double adjustSetpoint(double delta)
  {
    double setpoint = getSetpoint();
    setpoint += (delta * 0.25);

    // if(delta < 0 && setpoint < -0.5)
    // {
    //   setpoint = -0.5;
    // }
    
    setSetpoint(setpoint);
    return setpoint;
  }
}
