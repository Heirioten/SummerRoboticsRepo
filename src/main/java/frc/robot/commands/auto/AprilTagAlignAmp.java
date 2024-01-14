// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import javax.swing.text.html.Option;

import org.opencv.photo.Photo;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class AprilTagAlignAmp extends Command {
    
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    PIDController xController;
    PIDController yController;
    PIDController yawController;

    int targetId;

    public AprilTagAlignAmp(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, int id, double p, double i, double d, double ap, double ai, double ad) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        xController = new PIDController(p, i, d);
        yController = new PIDController(p, i, d);
        yawController = new PIDController(ap, ai, ad);
        this.targetId = id;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Optional<PhotonTrackedTarget> targetOptional = getTarget();
        if(targetOptional.isPresent()) {
            PhotonTrackedTarget target = targetOptional.get();
            target.getBestCameraToTarget();
        }
    }

    public Optional<PhotonTrackedTarget> getTarget() {
        if(visionSubsystem.hasTargets()) {
            for(int i = 0; i < visionSubsystem.getTargets().size(); i++) {
                int id = visionSubsystem.getTargets().get(i).getFiducialId();
                if(id == targetId) {
                    return Optional.of(visionSubsystem.getTargets().get(i));
                }
            }
        }

        return Optional.empty();
    }
}
