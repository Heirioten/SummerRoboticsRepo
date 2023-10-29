package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants 
{
  public static class OperatorConstants 
  {

    public static final double kDistancePerPulse = 0.0053218923153;
    public static final int kDriverControllerPort = 0;
    public static final double kDeadzone = 0.05;
    public static final double kExtensionDeadzone = 0.25;
    public static final int kNeo550CurrentLimit = 20;
    public static final double kDriveRateLimit = 2.0;
    public static final double kDriveSpeedDivisor = 1.5;
    public static final double kDriveTurnDivisor = 2.5;

    public static final float kGripperForwardLimit = 17.0f;
    public static final float kGripperReverseLimit = -2f;

    public static final double kPivotSpeed = 2.0;
    public static final double kExtensionSpeed_IN = 1;
    public static final double kExtensionSpeed_OUT = 3;
    public static final double kGripperSpeed = 0.5;


    // ALL OF THESE need to be measured
    public static final double kDriveGearing = 10.71;
    public static final double kMOI = 7.5;
    public static final double kMass = 60.0;

    public static final double kS = 0.22;
    public static final double kV = 1.98;
    public static final double kA = 0.2;

    public static final double kP = 5.0;

    public static final double kTrackWidth = 1;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

    public static final double kB = 2.0;
    public static final double kZeta = 0.7;

    public static final double kMaxAutoVel = 2.0;
    public static final double kMaxAutoAccel = 1.0;
  }
}