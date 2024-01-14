package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTrajectoryCommand extends SequentialCommandGroup {

  DriveSubsystem driveSubsystem;

  public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, PathPlannerPath path) {
    Pose2d pose = path.getPreviewStartingHolonomicPose();
    driveSubsystem.setPose(pose);

    addCommands(AutoBuilder.buildAuto("auto0"));

    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }
}
