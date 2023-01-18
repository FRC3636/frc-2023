package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
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
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) return;

        PhotonTrackedTarget target = result.getBestTarget();

        Pose3d robotPos = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), FieldConstants.aprilTags.get(target.getFiducialId()), new Transform3d());

        RobotContainer.field.setRobotPose(robotPos.toPose2d());
    }
}
