package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTrajectoryCommand extends SequentialCommandGroup {

    DriveSubsystem driveSubsystem;
    
    public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, PathPlannerTrajectory trajectory, SwerveAutoBuilder swerveAuto) {
        addCommands(
            swerveAuto.followPath(trajectory)
        );
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
}
