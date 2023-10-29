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


// drivebase is most complicated part of the robot. 6 wheels, 4 motors, 4 encoders, 2 gearboxes 1 gyro -- its kinda just a lot of stuff to manage.

// on our robot, there are two motors per side. the two motors are connected onto the same output belt.
public class DriveSubsystem extends SubsystemBase {
  
  CANSparkMax fl, fr, bl, br; // our motors
  RelativeEncoder encoderFl, encoderFr, encoderBl, encoderBr; // our encoders
  MotorControllerGroup left, right; // simplification of four motors
  DifferentialDrive drive; // useful for calculating arcadeDrive and some other stuff

  Field2d field; // field, useful for testing robot in simulation. image of field, and animated robot position on it

  PigeonIMU pigeon; // gyroscope

  DifferentialDriveOdometry odometry; // keeps track of where we are

  DifferentialDrivetrainSim drivetrainSim; // simulates physics
  
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
    encoderFl.setPositionConversionFactor(OperatorConstants.kDistancePerPulse); // how far does the robot move with 1 tick of the encoder? 42 tick per full rotation encoder (encoder is on the motor output shaft), 10.71:1 gear reduction, 6 inch diameter wheels.
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

    odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0); // initial position -- x=0, y=0, theta=0

    field = new Field2d();

    drivetrainSim = new DifferentialDrivetrainSim(DCMotor.getNEO(4), // 4 motors on drivetrain
      OperatorConstants.kDriveGearing, // 10.71:1
      OperatorConstants.kMOI, // i didnt actually calculate this
      OperatorConstants.kMass, // i didnt actually measure this
      Units.inchesToMeters(3), // 6 inch diameter / 3 inch radius
      OperatorConstants.kTrackWidth, // i didnt actually measure this
      VecBuilder.fill(0.001, 0.01, 0.001, 0.1, 0.1, 0.005, 0.005) // lets add some noise, just for fun i guess (probably actually to make sure your code doesnt do a freak out in the real world)
    );
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getYaw()), leftEncoderAverage(), rightEncoderAverage()); // updates your position based on gyro angle, how distance travelled of the left side of your drivetrain, and on the right
    Pose2d odometryPose = odometry.getPoseMeters();
    field.setRobotPose(odometryPose); // tell the field where we are
    SmartDashboard.putData(field); // tell the dashboard about the field
    SmartDashboard.putNumber("Field X", odometryPose.getX()); // lets get some numbers too i guess
    SmartDashboard.putNumber("Field Y", odometryPose.getY());
  }

  @Override
  public void simulationPeriodic() {

    drivetrainSim.setInputs(left.get() * RobotController.getInputVoltage(), right.get() * RobotController.getInputVoltage()); // give the sim voltages on the left and right side
    drivetrainSim.update(0.02); // 20ms since last update
    encoderFl.setPosition(drivetrainSim.getLeftPositionMeters());
    encoderFr.setPosition(drivetrainSim.getRightPositionMeters());
    encoderBl.setPosition(drivetrainSim.getLeftPositionMeters());
    encoderBr.setPosition(drivetrainSim.getRightPositionMeters());

    pigeon.setYaw(drivetrainSim.getHeading().getDegrees());
    SmartDashboard.putNumber("Sim Gyro", pigeon.getYaw());
    
  }

  // now a bunch of short methods. most of these exist for the sake of following trajectories with WPILib's built in trajectory-follower, the RAMSETE Controller.

  public void resetEncoders() { // in case we wanna be at zero again
    encoderFl.setPosition(0);
    encoderFr.setPosition(0);
    encoderBl.setPosition(0);
    encoderBr.setPosition(0);
  }

  public void resetGyro() { // v.s.
    pigeon.setYaw(0);
  }

  public void resetOdometry() { // v.s.
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

  // now methods
  public void arcadeDrive(double y, double omega)
  {
    drive.arcadeDrive(y, omega);
  }

  public void tankDrive(double ly, double ry) {
    left.set(ly);
    right.set(ry);
    drive.feed(); // the DifferentialDrive object FREAKS THE HELL OUT if we dont go through it directly and we forget to tell it we're not going through it anymore. this freak out is called MotorSafety, and it stops your motor from moving
    // the feed() method tells it "by the way, im doing my own thing now"
    // imagine you tell your mom that you're gonna call every 30 minutes, but its a 1 hour drive home, and on your last call before leaving you forget to mention that you're going home now. MotorSafety is your mother freaking out.
  }


  // this one is only ever used in trajectory following
  public void tankDriveVolts(double leftV, double rightV) {
    // left.setVoltage(leftV); 
    // right.setVoltage(rightV);
    left.set(leftV / RobotController.getBatteryVoltage()); // normally i wouldnt do it this way, but simulated CANSparkMaxes freak out when you use setVoltage instead of set. this is DEFINITELY bad practice
    right.set(rightV / RobotController.getBatteryVoltage());
    drive.feed(); // L168-170
  }
}

// now lets move on to the rest of RobotContainer, starting at getAutonomousCommand