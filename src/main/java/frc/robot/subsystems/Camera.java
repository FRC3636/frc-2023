package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera implements Subsystem {

    private PhotonCamera camera;

    public Camera() {
        camera = new PhotonCamera("test-camera");
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);
    }
    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) {return;}

        System.out.println("updating");

        PhotonTrackedTarget target =  result.getBestTarget();

        Pose3d robotPos = FieldConstants.aprilTags.get(target.getFiducialId()).transformBy(target.getBestCameraToTarget());

//        RobotContainer.testField.setRobotPose(robotPos.toPose2d());
    }
}
