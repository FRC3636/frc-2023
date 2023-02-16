package frc.robot.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class PoseEstimation {
    private PhotonVisionBackend photonVision;
    private SwerveDrivePoseEstimator poseEstimator;

    private GenericEntry usePhotonVisionEntry = RobotContainer.autoTab.add("Use PhotonVision", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    public PoseEstimation() {
        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, RobotContainer.drivetrain.getRotation(), RobotContainer.drivetrain.getModulePositions(), new Pose2d());

        try {
            photonVision = new PhotonVisionBackend();
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.println("PoseEstimation initialized");
    }

    public void periodic() {
        if (usePhotonVisionEntry.getBoolean(false)) {
            photonVision.getMeasurement().ifPresent((measurement) -> {
                poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation);
            });
        }

        RobotContainer.field.setRobotPose(getPose());
    }

    public void updateOdometry(Rotation2d gyro, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotContainer.drivetrain.getRotation(), RobotContainer.drivetrain.getModulePositions(), pose);
    }
}