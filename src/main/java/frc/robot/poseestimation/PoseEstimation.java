package frc.robot.poseestimation;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class PoseEstimation {
    private SwerveDrivePoseEstimator poseEstimator;

    private Translation2d carpetBias = new Translation2d(1, 0).times(DriveConstants.CARPET_BIAS);

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

        if(Robot.isReal()) {
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
        else {
            backends = new VisionBackend[0];
            backendToggles = new GenericEntry[0];
        }
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
        // carpet bias correction
        for (int i = 0; i < DriveConstants.MODULE_POSITIONS.length; i++) {
            Translation2d velocity = modulePositionToTranslation(modulePositions[i]);

            Translation2d wheelRelativeCarpetBias = carpetBias
                    .rotateBy(gyro.unaryMinus())
                    .rotateBy(DriveConstants.MODULE_ROTATIONS[i].unaryMinus());
            velocity = velocity.times(1 + translationDot(velocity, wheelRelativeCarpetBias));

            modulePositions[i] = translationToModulePosition(velocity);
        }

        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getEstimatedVelocity() {
        double now = Timer.getFPGATimestamp();

        Translation2d current = poseHistory.getSample(now).orElseGet(Pose2d::new).getTranslation();
        Translation2d previous = poseHistory.getSample(now - DIFFERENTIATION_TIME).orElseGet(Pose2d::new).getTranslation();

        return current.minus(previous).div(DIFFERENTIATION_TIME);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotContainer.drivetrain.getRotation(), RobotContainer.drivetrain.getModulePositions(), pose);
    }

    private void addVisionMeasurement(VisionBackend.Measurement measurement) {
        poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation);
    }

    private static Translation2d modulePositionToTranslation(SwerveModulePosition state) {
        return new Translation2d(
                state.distanceMeters,
                state.angle
        );
    }

    private static SwerveModulePosition translationToModulePosition(Translation2d translation) {
        return new SwerveModulePosition(
                translation.getNorm(),
                new Rotation2d(translation.getX(), translation.getY())
        );
    }

    private static double translationDot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
}