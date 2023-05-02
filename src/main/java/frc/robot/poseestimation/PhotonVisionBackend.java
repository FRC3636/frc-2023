package frc.robot.poseestimation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.Optional;

public class PhotonVisionBackend extends VisionBackend {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionBackend(String name, Transform3d camera_transform) throws IOException {
        camera = new PhotonCamera(name);
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, camera_transform);
    }

    @Override
    public Optional<PoseEstimation.Measurement> getMeasurement() {
        return poseEstimator.update().flatMap((result) -> {
            RobotContainer.field.getObject("Vision Measurement " + camera.getName()).setPose(result.estimatedPose.toPose2d());

            if (result.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() > Constants.VisionConstants.DISTANCE_FILTER || result.targetsUsed.get(0).getPoseAmbiguity() > Constants.VisionConstants.AMBIGUITY_FILTER) {
                return Optional.empty();
            }

            // Reject pose estimates outside the field
            if (result.estimatedPose.toPose2d().getX() < 0 || result.estimatedPose.toPose2d().getX() > Constants.FieldConstants.fieldLength ||
                    result.estimatedPose.toPose2d().getY() < 0 || result.estimatedPose.toPose2d().getY() > Constants.FieldConstants.fieldWidth) {
                return Optional.empty();
            }

            return Optional.of(new PoseEstimation.Measurement(
                    result.timestampSeconds,
                    result.estimatedPose,
                    Constants.VisionConstants.PHOTON_VISION_STD_DEV.forMeasurement(result.targetsUsed.get(0).getBestCameraToTarget().getX(), result.targetsUsed.size())
            ));
        });
    }

    public interface StandardDeviation {
        Vector<N3> forMeasurement(double distance, int count);
    }
}
