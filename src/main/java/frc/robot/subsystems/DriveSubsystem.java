// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  CANSparkMax fl, fr, bl, br;
  RelativeEncoder encoderFl, encoderFr, encoderBl, encoderBr;
  MotorControllerGroup left, right;
  DifferentialDrive drive;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
    fl = new CANSparkMax(1, MotorType.kBrushless);
    fr = new CANSparkMax(2, MotorType.kBrushless);
    bl = new CANSparkMax(3, MotorType.kBrushless);
    br = new CANSparkMax(4, MotorType.kBrushless);
    encoderFl = fl.getEncoder();
    encoderFr = fr.getEncoder();
    encoderBl = bl.getEncoder();
    encoderBr = br.getEncoder();
    fl.restoreFactoryDefaults();
    fr.restoreFactoryDefaults();
    bl.restoreFactoryDefaults();
    br.restoreFactoryDefaults();

    left = new MotorControllerGroup(fl, bl);
    right = new MotorControllerGroup(fr, br);
    left.setInverted(true);

    drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double y, double omega)
  {
    drive.arcadeDrive(y, omega);
  }

  public void tankDrive(double ly, double ry) {
    left.set(ly);
    right.set(ry);
  }
}
