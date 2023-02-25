package frc.robot.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVisionBackend extends VisionBackend {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionBackend() throws IOException {
        camera = new PhotonCamera("arducam");
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, Constants.VisionConstants.PHOTONVISION_TRANSFORM);
    }

    @Override
    public Optional<Measurement> getMeasurement() {
        return poseEstimator.update().map((result) -> {
            return new Measurement(
                result.timestampSeconds,
                result.estimatedPose,
                Constants.VisionConstants.PHOTONVISION_STD_DEV
            );
        });
    }
}