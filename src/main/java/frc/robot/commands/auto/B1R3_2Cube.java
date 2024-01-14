package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class B1R3_2Cube extends SwerveTrajectoryCommand {

  public B1R3_2Cube(DriveSubsystem driveSubsystem) {
    super(driveSubsystem, getAutonomousTrajectory());
  }

  public static PathPlannerPath getAutonomousTrajectory() {
    var allianceOptional = DriverStation.getAlliance();
    Alliance alliance = null;
    if (allianceOptional.isPresent()) {
      alliance = allianceOptional.get();
    } else {
      alliance = Alliance.Blue;
    }

    if (alliance == Alliance.Blue) {
      return PathPlannerPath.fromPathFile("path0");
    } else return PathPlannerPath.fromPathFile("path0");
  }
}
