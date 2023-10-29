package frc.robot;

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
  }
}