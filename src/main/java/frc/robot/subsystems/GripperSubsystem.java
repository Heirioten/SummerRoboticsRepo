// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class GripperSubsystem extends SubsystemBase {
  
  CANSparkMax gripper;
  RelativeEncoder encoder;

  public GripperSubsystem() 
  {
    gripper = new CANSparkMax(7, MotorType.kBrushless);
    encoder = gripper.getEncoder();    
    gripper.restoreFactoryDefaults();
    gripper.enableSoftLimit(SoftLimitDirection.kReverse, true);
    gripper.setSoftLimit(SoftLimitDirection.kReverse, OperatorConstants.kGripperReverseLimit);
    gripper.enableSoftLimit(SoftLimitDirection.kForward, true);
    gripper.setSoftLimit(SoftLimitDirection.kForward, OperatorConstants.kGripperForwardLimit);
    gripper.setSmartCurrentLimit(OperatorConstants.kNeo550CurrentLimit);
  }

  @Override
  public void periodic() 
  {
  }

  public enum GripperState 
  {
    OPEN, 
    CLOSE
  }

  public void setState(GripperState state)
  {
    if(state == GripperState.OPEN)
    {
      gripper.set(-0.5);
    }
    else
    {
      gripper.set(0.5);
    }
  }


}
