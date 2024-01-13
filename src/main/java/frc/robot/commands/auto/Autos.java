// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class Autos {

    private static SwerveAutoBuilder autoBuilder;
    private static HashMap<String, Command> eventMap;
    private static DriveSubsystem driveSubsystem;
    
    public static void constructAutoBuilder(DriveSubsystem _driveSubsystem) {
        driveSubsystem = _driveSubsystem;

        builtEventMap();

        autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::setPose,
            new PIDConstants(OperatorConstants.kP, 0, 0),
            new PIDConstants(OperatorConstants.kP, 0, 0),
            driveSubsystem::driveChassisSpeeds,
            eventMap,
            false, // Automatically transform trajectories for blue alliance to work on red alliance side
            driveSubsystem
        );
    }

    public static void builtEventMap() {
        eventMap =  new HashMap<>(
            Map.ofEntries(
                Map.entry("deployArm", new PrintCommand("deployArm")),
                Map.entry("grab", new PrintCommand("grab")),
                Map.entry("release", new PrintCommand("release"))
            )
        );
    }

    public static Command B1R3_2Cube() {
        return new B1R3_2Cube(driveSubsystem, autoBuilder);
    }
}
