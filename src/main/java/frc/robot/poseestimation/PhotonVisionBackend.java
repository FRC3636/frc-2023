package frc.robot.poseestimation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.Optional;

public class PhotonVisionBackend extends VisionBackend {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionBackend(String name) throws IOException {
        camera = new PhotonCamera(name);
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, Constants.VisionConstants.PHOTONVISION_TRANSFORM);
    }

    @Override
    public Optional<Measurement> getMeasurement() {
        return poseEstimator.update().flatMap((result) -> {
            if (result.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() > Constants.VisionConstants.DISTANCE_FILTER || result.targetsUsed.get(0).getPoseAmbiguity() > Constants.VisionConstants.AMBIGUITY_FILTER) {
                return Optional.empty();
            }


            RobotContainer.field.getObject("Vision Measurement").setPose(result.estimatedPose.toPose2d());

            return Optional.of(new Measurement(
                    result.timestampSeconds,
                    result.estimatedPose,
                    Constants.VisionConstants.PHOTONVISION_STD_DEV
            ));
        });
    }
}
