// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    PhotonPipelineResult result;
    List<PhotonTrackedTarget> targets;
    PhotonTrackedTarget bestTarget;

    VisionSystemSim sim;

    public VisionSubsystem() {
        camera = new PhotonCamera("front");
        try {
            poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.kTransformToRobot);
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();

        targets = result.getTargets();
        bestTarget = result.getBestTarget();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return targets;
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    public boolean hasTargets() {
        return result.hasTargets();
    }
}