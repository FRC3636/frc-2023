package frc.robot.poseestimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class PoseEstimation {
    private SwerveDrivePoseEstimator poseEstimator;

    private VisionBackend[] backends;
    private GenericEntry[] backendToggles;

    private TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5);

    private static final double DIFFERENTIATION_TIME = Robot.kDefaultPeriod;

    public PoseEstimation() {
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            RobotContainer.drivetrain.getRotation(),
            RobotContainer.drivetrain.getModulePositions(),
            new Pose2d(),
            Constants.DriveConstants.ODOMETRY_STD_DEV,
            VecBuilder.fill(0, 0, 0) // will be overwritten for each measurement
        );

        backends = new VisionBackend[2];
        backendToggles = new GenericEntry[2];

        try {
            backends[0] = new PhotonVisionBackend("arducam");
            backendToggles[0] = RobotContainer.autoTab.add("VisionBackend/PV", true).getEntry();
        } catch (Exception e) {
            System.out.println("Failed to initialize PhotonVision");
            e.printStackTrace();
        }

        backends[1] = new LimelightBackend();
        backendToggles[1] = RobotContainer.autoTab.add("VisionBackend/LL", true).getEntry();
    }

    public void periodic() {
        poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());

        for (int i = 0; i < backends.length; i++) {
            if (backendToggles[i].getBoolean(false)) {
                backends[i].getMeasurement().ifPresent(this::addVisionMeasurement);
            }
        }

        RobotContainer.field.setRobotPose(getEstimatedPose());
    }

    public void updateOdometry(Rotation2d gyro, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getEstimatedVelocity() {
        double now = Timer.getFPGATimestamp();

        Translation2d current = poseHistory.getSample(now).get().getTranslation();
        Translation2d previous = poseHistory.getSample(now - DIFFERENTIATION_TIME).get().getTranslation();

        return current.minus(previous).div(DIFFERENTIATION_TIME);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotContainer.drivetrain.getRotation(), RobotContainer.drivetrain.getModulePositions(), pose);
    }

    private void addVisionMeasurement(VisionBackend.Measurement measurement) {
        poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation);
    }
}