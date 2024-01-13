// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.SimPhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera;
    SimPhotonCamera cameraSim;
    private static PhotonPoseEstimator poseEstimator;
    private static PhotonPipelineResult result;
    private static List<PhotonTrackedTarget> targets;
    private static PhotonTrackedTarget bestTarget;

    SimVisionSystem sim;

    public VisionSubsystem() {
        // if(Robot.isReal())
        camera = new PhotonCamera("front");

        // else {
            sim = new SimVisionSystem("front",
            70,
            VisionConstants.kTransformToRobot,
            20,
            640,
            480,
            1.0);

            // cameraSim = new SimPhotonCamera("cameraSim");
        // }

        // all these properties copied from example
        // cameraProperties = new SimCameraProperties();
        // cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        // cameraProperties.setCalibError(0.25, 0.08);
        // cameraProperties.setFPS(20);
        // cameraProperties.setAvgLatencyMs(35);
        // cameraProperties.setLatencyStdDevMs(5);
        // cameraSim.enableDrawWireframe(true);
        
        try {
            poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
                PoseStrategy.AVERAGE_BEST_TARGETS,
                camera,
                VisionConstants.kTransformToRobot);

            sim.addVisionTargets(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();

        if(hasTargets()) {
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
        }

        sim.processFrame(DriveSubsystem.pose);
    }

    public static List<PhotonTrackedTarget> getTargets() {
        return targets;
    }

    public static PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    public static boolean hasTargets() {
        return result.hasTargets();
    }

    public static Optional<EstimatedRobotPose> getVisionPose() {
        return poseEstimator.update(result);
    }
}