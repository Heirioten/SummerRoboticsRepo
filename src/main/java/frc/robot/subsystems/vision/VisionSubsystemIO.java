// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface VisionSubsystemIO {
    @AutoLog
    public class VisionSubsystemIOInputs {
        Pose3d estimatedPose = new Pose3d();
        double estimatedPoseTimestamp = 0.0;
        int[] tags = new int[] {};
        double lastCameraTimestamp = 0.0;
        PhotonPipelineResult result = new PhotonPipelineResult();
    }

    public default void updateInputs(VisionSubsystemIOInputs inputs) {

    }

    public default boolean hasTargets() {
        return false;
    }

    public default List<PhotonTrackedTarget> getTargets() {
        return null;
    }

    public default PhotonTrackedTarget getBestTarget() {
        return null;
    }


}
