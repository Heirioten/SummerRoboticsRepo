// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  
  private final DriveSubsystemIO io;
  DriveSubsystemIOInputsAutoLogged inputs = new DriveSubsystemIOInputsAutoLogged();

  Field2d field;
  DifferentialDriveOdometry odometry;

  public DriveSubsystem(DriveSubsystemIO io) 
  {
    this.io = io;
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);

    field = new Field2d();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("DriveSubsystem", inputs);

    Logger.getInstance().recordOutput("Odometry/Pose", getPose());
    
    odometry.update(Rotation2d.fromDegrees(getYaw()), leftEncoderAverage(), rightEncoderAverage());
  }

  // Drive methods
  public void arcadeDrive(double y, double omega)
  {
    var speeds = DifferentialDrive.arcadeDriveIK(y, omega, true);
    tankDrive(speeds.left, speeds.right);
  }

  public void tankDrive(double left, double right) {
    tankDriveVolts(left * RobotController.getBatteryVoltage(), right * RobotController.getBatteryVoltage());
  }

  public void tankDriveVolts(double leftV, double rightV) {
    io.setLeftVoltage(leftV);
    io.setRightVoltage(rightV);
  }




  // Getters / Setters

  public void setPose(double x, double y, double yaw) {
    odometry.resetPosition(new Rotation2d(yaw), leftEncoderAverage(), rightEncoderAverage(), new Pose2d(x, y, new Rotation2d(yaw)));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    var speeds = new DifferentialDriveWheelSpeeds(inputs.leftVelocity, inputs.rightVelocity);

    return speeds;
  }

  public double getYaw() {
    return inputs.gyroYaw;
  }

  public double leftEncoderAverage() {
    return inputs.leftEncoderAverage;
  }

  public double rightEncoderAverage() {
    return inputs.rightEncoderAverage;
  }

  // not sure why we need this one but trajectory tutorial says we do and i havent read it all yet
  public double getEncoderAverage() {
    return (leftEncoderAverage() + rightEncoderAverage()) / 2;
  }
}
