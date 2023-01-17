package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera implements Subsystem {

    private final PhotonCamera camera;

    public Camera() {
        camera = new PhotonCamera("test-camera");
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return;
        }

        System.out.println("updating");

        PhotonTrackedTarget target = result.getBestTarget();

        Pose3d robotPos = Constants.Field.aprilTags.get(target.getFiducialId()).transformBy(target.getBestCameraToTarget());

//        RobotContainer.testField.setRobotPose(robotPos.toPose2d());
    }
}
