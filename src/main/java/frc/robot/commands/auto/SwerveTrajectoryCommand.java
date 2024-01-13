package frc.robot.commands.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTrajectoryCommand extends SequentialCommandGroup {

    DriveSubsystem driveSubsystem;
    
    public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, PathPlannerTrajectory trajectory, SwerveAutoBuilder swerveAuto) {
        Pose2d pose = trajectory.getInitialHolonomicPose();
        driveSubsystem.setPose(pose);

        
        addCommands(
            swerveAuto.followPath(trajectory)
        );
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, PathPlannerTrajectory trajectory) {
        this(driveSubsystem, trajectory,
            new SwerveAutoBuilder(
                driveSubsystem::getPose,
                driveSubsystem::setPose,
                new PIDConstants(OperatorConstants.kP, 0, 0),
                new PIDConstants(OperatorConstants.kP, 0, 0),
                driveSubsystem::driveChassisSpeeds,
                new HashMap<>(),
                driveSubsystem
        ));
    }
}
