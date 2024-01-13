package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;


/** Add your docs here. */
public class B1R3_2Cube extends SwerveTrajectoryCommand {

    
    public B1R3_2Cube(DriveSubsystem driveSubsystem, SwerveAutoBuilder autoBuilder) {
        super(
            driveSubsystem, 
            getAutonomousTrajectory(),
            autoBuilder
        );
    }

    public static PathPlannerTrajectory getAutonomousTrajectory() {
        if(DriverStation.getAlliance() == Alliance.Blue) {
            return PathPlanner.loadPath("Blue1_2Cube_0", OperatorConstants.kMaxAutoVel, OperatorConstants.kMaxAutoAccel);
        } else
            return PathPlanner.loadPath("Red3_2Cube_0", OperatorConstants.kMaxAutoVel, OperatorConstants.kMaxAutoAccel);
    }
}
