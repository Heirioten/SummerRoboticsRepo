package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants 
{
  public static class OperatorConstants 
  {
    // Encoder Conversions
    public static final double kDistancePerPulse = (.1524 * Math.PI) / 10.71;
    public static final double kVelocityConversionFactor = 0.0865911847/60.0;

    // User controller settings
    public static final int kDriverControllerPort = 0;
    public static final double kDeadzone = 0.05;
    
    public static final double kDriveRateLimit = 2.0;
    public static final double kDriveSpeedDivisor = 1.5;
    public static final double kDriveTurnDivisor = 2.5;

    // Drivetrain Limits
    public static final double kMaxSpeed = 3.81;
    public static final double kMaxAngVel = 3.615;


    public static final double kDriveGearing = 10.71;

    public static final double kMOI = 7.5; // unmeasured, arbitrary value
    public static final double kMass = 60.0; // ^

    // Measured in SysID
    public static final double kS = 0.120;
    public static final double kV = 1.36;
    public static final double kA = 0.236;

    public static final double kP = 0.146;

    public static final double kTrackWidth = .7112;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Ramsete settings
    public static final double kB = 2.0;
    public static final double kZeta = 0.7;

    public static final double kMaxAutoVel = 2.0;
    public static final double kMaxAutoAccel = 1.0;
  }

  public static class VisionConstants {
    public static final Transform3d kTransformToRobot = new Transform3d(new Translation3d(0, 0, 0.75), new Rotation3d());
  }
}