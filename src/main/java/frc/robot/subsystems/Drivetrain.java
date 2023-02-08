package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Drivetrain implements Subsystem {
    private final WPI_TalonFX leftMotor1, leftMotor2, rightMotor1, rightMotor2;


    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
            new Pose2d());
    private final DifferentialDrive drivetrain;
    private final AHRS navX = new AHRS();


    public Drivetrain() {
        leftMotor1 = new WPI_TalonFX(Constants.Drivetrain.MOTOR_LEFT_1);
        leftMotor2 = new WPI_TalonFX(Constants.Drivetrain.MOTOR_LEFT_2);
        rightMotor1 = new WPI_TalonFX(Constants.Drivetrain.MOTOR_RIGHT_1);
        rightMotor2 = new WPI_TalonFX(Constants.Drivetrain.MOTOR_RIGHT_2);

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);
        leftMotor1.setInverted(false);
        leftMotor2.setInverted(false);

        drivetrain = new DifferentialDrive(leftMotor1, rightMotor1);
    }

    @Override
    public void periodic() {
 
//        Pose2d pose = RobotContainer.camera.getRobotPose();
//        if(pose != null) {
//            // resetOdometryTo(pose);
//        }

        SmartDashboard.putNumber("Robot X", getPose().getX());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftMotor1.getSelectedSensorVelocity()
                        / Constants.Drivetrain.SENSOR_UNITS_PER_METER,
                rightMotor1.getSelectedSensorVelocity()
                        / Constants.Drivetrain.SENSOR_UNITS_PER_METER);
    }

    public void resetOdometryTo(Pose2d pose2d) {
        resetEncoders();
        odometry.resetPosition(navX.getRotation2d(), 0, 0, pose2d);
    }

    public void resetEncoders() {
        leftMotor1.setSelectedSensorPosition(0);
        rightMotor1.setSelectedSensorPosition(0);
        navX.reset();
    }

    public void stop() {
        leftMotor1.set(ControlMode.PercentOutput, 0);
        rightMotor1.set(ControlMode.PercentOutput, 0);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }

    public void tankDrive(double left, double right) {
        drivetrain.tankDrive(left, right);
    }

    public void tankDriveWithRawVoltage(double leftV, double rightV) {
        leftMotor1.setVoltage(leftV);
        rightMotor1.setVoltage(rightV);
    }
}
