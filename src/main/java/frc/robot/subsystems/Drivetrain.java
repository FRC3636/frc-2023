package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain implements Subsystem {
    private final Spark motorLeft = new Spark(Constants.Drivetrain.MOTOR_LEFT);
    private final Spark motorRight = new Spark(Constants.Drivetrain.MOTOR_RIGHT);

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
            new Pose2d());
    private final DifferentialDrive drivetrain;
    private final AHRS navX = new AHRS();

    private final Encoder leftEncoder = new Encoder(
            Constants.Drivetrain.ENCODER_LEFT_PORT_A,
            Constants.Drivetrain.ENCODER_LEFT_PORT_B,
            false
    );
    private final Encoder rightEncoder = new Encoder(
            Constants.Drivetrain.ENCODER_RIGHT_PORT_A,
            Constants.Drivetrain.ENCODER_RIGHT_PORT_B,
            true
    );

    public Drivetrain() {
        motorRight.setInverted(false);
        motorLeft.setInverted(true);

        drivetrain = new DifferentialDrive(motorLeft, motorRight);

        double distancePerPulse = Constants.Drivetrain.WHEEL_CIRCUMFERENCE
                / (Constants.Drivetrain.PULSES_PER_REVOLUTION / 4f); // magic number that by all means shouldn't be here
                                                                     // but is

        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
    }

    @Override
    public void periodic() {
        double dl = leftEncoder.getDistance();
        double dr = rightEncoder.getDistance();
        odometry.update(navX.getRotation2d(), dl, dr);

        RobotContainer.field.setRobotPose(getPose());

        Pose2d pose = RobotContainer.camera.getRobotPose();
        if(pose != null) {
            resetOdometryTo(pose);
        }

        SmartDashboard.putNumber("left Encoder", leftEncoder.getDistance());
        SmartDashboard.putNumber("right Encoder", rightEncoder.getDistance());

        SmartDashboard.putNumber("Robot X", getPose().getX());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometryTo(Pose2d pose2d) {
        resetEncoders();
        odometry.resetPosition(navX.getRotation2d(), 0, 0, pose2d);
    }

    private void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        navX.reset();
    }

    public void stop() {
        motorRight.set(0);
        motorLeft.set(0);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }

    public void tankDrive(double left, double right) {
        drivetrain.tankDrive(left, right);
    }

    public void tankDriveWithRawVoltage(double leftV, double rightV) {
        motorLeft.setVoltage(leftV);
        motorRight.setVoltage(rightV);
    }
}
