package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {

    PhotonCamera camera;

    public Camera() {
        camera = new PhotonCamera("arducam");
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);
    }

    @Override
    public void periodic() {
        if (getRobotPose() != null) {
            RobotContainer.field.getObject("Camera").setPose(getRobotPose());
        }
    }

    public Pose2d getRobotPose() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets())
            return new Pose2d();

        PhotonTrackedTarget target = result.getBestTarget();

        Pose2d cameraPos = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                Constants.FieldConstants.aprilTags.get(target.getFiducialId()), new Transform3d()).toPose2d();

        Constants.Robot.CAMERA_OFFSET.getTranslation().rotateBy(cameraPos.getRotation());

        return new Pose2d(
                cameraPos.getTranslation()
                        .minus(Constants.Robot.CAMERA_OFFSET.getTranslation().rotateBy(cameraPos.getRotation())),
                cameraPos.getRotation());
    }
}
