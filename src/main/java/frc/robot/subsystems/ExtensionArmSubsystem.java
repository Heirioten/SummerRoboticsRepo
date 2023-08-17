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


public class ExtensionArmSubsystem extends PIDSubsystem {
  
  CANSparkMax extension;
  RelativeEncoder encoder;
  
  /** Creates a new ExtensionArmSubsystem. */
  public ExtensionArmSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.5, 0, 0));
    
    extension = new CANSparkMax(6, MotorType.kBrushless);
    encoder = extension.getEncoder();

    extension.restoreFactoryDefaults();

    enable();
  }

  @Override
  public void useOutput(double output, double setpoint) 
  {
    //extension.set(output);
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
    setSetpoint(setpoint);
    return setpoint;
  }
}
