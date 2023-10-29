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

// subsystems provide our codes lowest level of control over our physical motors/other related items (sometimes limit switches/actuators/valves/encoders)

public class GripperSubsystem extends SubsystemBase {
  
  CANSparkMax gripper; // motor controller
  RelativeEncoder encoder; // encoder (measures the position of the motor)

  public GripperSubsystem() 
  {
    gripper = new CANSparkMax(7, MotorType.kBrushless); // controller ID 7, and our motor is a REV NEO, which is brushless
    encoder = gripper.getEncoder(); // even if you dont want to use the encoder in the code, CANSparkMax gets upset if you dont get call getEncoder
    gripper.restoreFactoryDefaults(); // good idea to wipe everything on every startup

    /* soft limit controls how far the motor can turn. if this specific motor goes too far, itll overwork itself because the gripper arms push into each other and the motor starts to smoke. if it goes too far int he other direction, it tries to push
    two pieces of metal together, and it overworks itself and starts to smoke */ 
    gripper.enableSoftLimit(SoftLimitDirection.kReverse, true);
    gripper.setSoftLimit(SoftLimitDirection.kReverse, OperatorConstants.kGripperReverseLimit);
    gripper.enableSoftLimit(SoftLimitDirection.kForward, true);
    gripper.setSoftLimit(SoftLimitDirection.kForward, OperatorConstants.kGripperForwardLimit);
    gripper.setSmartCurrentLimit(OperatorConstants.kNeo550CurrentLimit); // good idea to have a currentlimit even if you have a fuse
  }

  @Override
  public void periodic() 
  {
  }

  public enum GripperState 
  { // we got a little fancy while programmming this part
    OPEN, 
    CLOSE
  }

  public void setState(GripperState state)
  {
    if(state == GripperState.OPEN)
    {
      gripper.set(-OperatorConstants.kGripperSpeed); // this is how we control motors
    }
    else
    {
      gripper.set(OperatorConstants.kGripperSpeed);
    }
  }


}
