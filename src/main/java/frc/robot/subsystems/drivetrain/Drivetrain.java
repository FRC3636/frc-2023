// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain extends SubsystemBase {
    // Create MAXSwerveModules
    private final SwerveModule frontLeft;

    private final SwerveModule frontRight;

    private final SwerveModule rearLeft;

    private final SwerveModule rearRight;

    // The gyro sensor
    private final Gyro gyro = (RobotBase.isReal() ? new NavXGyro() : new SIMGyro());

    /**
     * Creates a new DriveSubsystem.
     */
    public Drivetrain() {
        if(RobotBase.isSimulation()) {
            this.frontLeft = new SIMSwerveModule(DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);
            this.frontRight = new SIMSwerveModule(DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);
            this.rearLeft = new SIMSwerveModule(DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);
            this.rearRight = new SIMSwerveModule(DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);
        }
        else {
            this.frontLeft = new MAXSwerveModule(
                    DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                    DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

            this.frontRight = new MAXSwerveModule(
                    DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

            this.rearLeft = new MAXSwerveModule(
                    DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                    DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                    DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

            this.rearRight = new MAXSwerveModule(
                    DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                    DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);
        }

        RobotContainer.swerveTab.addNumber("Front Left", frontLeft::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        RobotContainer.swerveTab.addNumber("Front Right", frontRight::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        RobotContainer.swerveTab.addNumber("Back Left", rearLeft::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        RobotContainer.swerveTab.addNumber("Back Right", rearRight::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);

        RobotContainer.swerveTab.addNumber("Gyro", () -> gyro.getAngle().getDegrees()).withWidget(BuiltInWidgets.kGraph);

    }



    @Override
    public void periodic() {
        RobotContainer.poseEstimation.updateOdometry(
            getRotation(),
            getModulePositions());

        gyro.update();
        frontLeft.update();
        frontRight.update();
        rearLeft.update();
        rearRight.update();

        logModuleStates("Swerve State", new SwerveModuleState[] {
            frontLeft.getState(),
                    frontRight.getState(),
                    rearLeft.getState(),
                    rearRight.getState()
        });

        logModuleStates("Swerve Set State", new SwerveModuleState[] {
                frontLeft.getSetState(),
                frontRight.getSetState(),
                rearLeft.getSetState(),
                rearRight.getSetState()
        });
    }

    private void logModuleStates(String key, SwerveModuleState[] states) {
        List<Double> dataArray = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            dataArray.add(states[i].angle.getRadians());
            dataArray.add(states[i].speedMetersPerSecond);
        }
       SmartDashboard.putNumberArray(key,
                dataArray.stream().mapToDouble(Double::doubleValue).toArray());
    }
    public Rotation2d getRotation() {
        Rotation2d rot = gyro.getAngle();
        
        if (DriveConstants.GYRO_REVERSED) {
            rot = rot.unaryMinus();
        }

        return rot;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };
    }

    /**
     * Method to drive the drivetrain using chassis speeds.
     *
     * @param speeds The chassis speeds.
     */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }
    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        rearLeft.resetEncoders();
        frontRight.resetEncoders();
        rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return gyro.getAngle();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate().getDegrees();
    }

}
