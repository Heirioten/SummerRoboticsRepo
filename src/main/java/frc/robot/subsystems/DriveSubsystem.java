// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;


public class DriveSubsystem extends SubsystemBase {
  
  CANSparkMax fl, fr, bl, br;
  RelativeEncoder encoderFl, encoderFr, encoderBl, encoderBr;
  MotorControllerGroup left, right;
  DifferentialDrive drive;

  Field2d field;

  PigeonIMU pigeon;

  DifferentialDriveOdometry odometry;

  DifferentialDrivetrainSim drivetrainSim;
  
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
    encoderFl.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);
    encoderFr.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);
    encoderBl.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);
    encoderBr.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);
    fl.restoreFactoryDefaults();
    fr.restoreFactoryDefaults();
    bl.restoreFactoryDefaults();
    br.restoreFactoryDefaults();

    left = new MotorControllerGroup(fl, bl);
    right = new MotorControllerGroup(fr, br);
    left.setInverted(true);

    drive = new DifferentialDrive(left, right);

    pigeon = new PigeonIMU(0);

    odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);

    field = new Field2d();

    drivetrainSim = new DifferentialDrivetrainSim(DCMotor.getNEO(4),
      OperatorConstants.kDriveGearing,
      OperatorConstants.kMOI,
      OperatorConstants.kMass,
      Units.inchesToMeters(3),
      OperatorConstants.kTrackWidth,
      VecBuilder.fill(0.001, 0.01, 0.001, 0.1, 0.1, 0.005, 0.005)
    );
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getYaw()), leftEncoderAverage(), rightEncoderAverage());
    Pose2d odometryPose = odometry.getPoseMeters();
    field.setRobotPose(odometryPose);
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Field X", odometryPose.getX());
    SmartDashboard.putNumber("Field Y", odometryPose.getY());
  }

  @Override
  public void simulationPeriodic() {

    drivetrainSim.setInputs(left.get() * RobotController.getInputVoltage(), right.get() * RobotController.getInputVoltage());
    drivetrainSim.update(0.02);
    encoderFl.setPosition(drivetrainSim.getLeftPositionMeters());
    encoderFr.setPosition(drivetrainSim.getRightPositionMeters());
    encoderBl.setPosition(drivetrainSim.getLeftPositionMeters());
    encoderBr.setPosition(drivetrainSim.getRightPositionMeters());

    pigeon.setYaw(drivetrainSim.getHeading().getDegrees());
    SmartDashboard.putNumber("Sim Gyro", pigeon.getYaw());
    
  }

  public void resetEncoders() {
    encoderFl.setPosition(0);
    encoderFr.setPosition(0);
    encoderBl.setPosition(0);
    encoderBr.setPosition(0);
  }

  public void resetGyro() {
    pigeon.setYaw(0);
  }

  public void resetOdometry() {
    odometry.resetPosition(new Rotation2d(0), leftEncoderAverage(), rightEncoderAverage(), getPose());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      ((encoderFl.getVelocity() + encoderBl.getVelocity()) / 2),
      ((encoderFr.getVelocity() + encoderBr.getVelocity()) / 2)
    );
  }

  public double getYaw() {
    return pigeon.getYaw();
  }

  public double leftEncoderAverage() {
    return (encoderFl.getPosition() + encoderBl.getPosition()) / 2;
  }

  public double rightEncoderAverage() {
    return (encoderFr.getPosition() + encoderBr.getPosition()) / 2;
  }

  // not sure why we need this one but trajectory tutorial says we do and i havent read it all yet
  public double getEncoderAverage() {
    return (leftEncoderAverage() + rightEncoderAverage()) / 4;
  }

  // drive methods
  public void arcadeDrive(double y, double omega)
  {
    drive.arcadeDrive(y, omega);
  }

  public void tankDrive(double ly, double ry) {
    left.set(ly);
    right.set(ry);
    drive.feed();
  }

  public void tankDriveVolts(double leftV, double rightV) {
    // left.setVoltage(leftV);
    // right.setVoltage(rightV);
    left.set(leftV / RobotController.getBatteryVoltage());
    right.set(rightV / RobotController.getBatteryVoltage());
    drive.feed();
  }
}
