// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.DriveConstants
import frc.robot.RobotContainer
import frc.robot.subsystems.drivetrain.SwerveModules.States

class Drivetrain : SubsystemBase() {
    // Create MAXSwerveModules
    private var swerveModules: SwerveModules

    // The gyro sensor
    private val gyro: Gyro

    init {
        if (RobotBase.isSimulation()) {
            swerveModules = SwerveModules(
                    SIMSwerveModule(DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET),
                    SIMSwerveModule(DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET),
                    SIMSwerveModule(DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET),
                    SIMSwerveModule(DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET)
            )
        } else {
            swerveModules = SwerveModules(
                    MAXSwerveModule(
                            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
                    ),
                    MAXSwerveModule(
                            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
                    ),
                    MAXSwerveModule(
                            DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                            DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                            DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET
                    ),
                    MAXSwerveModule(
                            DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                            DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                            DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET
                    )
            )
        }
        gyro = if (RobotBase.isReal()) NavXGyro() else SIMGyro(swerveModules)
        RobotContainer.swerveTab.addNumber("Front Left", swerveModules.frontLeft::swerveEncoderPosition).withWidget(BuiltInWidgets.kGraph)
        RobotContainer.swerveTab.addNumber("Front Right", swerveModules.frontRight::swerveEncoderPosition).withWidget(BuiltInWidgets.kGraph)
        RobotContainer.swerveTab.addNumber("Back Left", swerveModules.rearLeft::swerveEncoderPosition).withWidget(BuiltInWidgets.kGraph)
        RobotContainer.swerveTab.addNumber("Back Right", swerveModules.rearRight::swerveEncoderPosition).withWidget(BuiltInWidgets.kGraph)
        RobotContainer.swerveTab.addNumber("Gyro") { gyro.angle.radians }.withWidget(BuiltInWidgets.kGraph)
    }

    override fun periodic() {
        RobotContainer.poseEstimation.updateOdometry(
                rotation,
                modulePositions
        )
        gyro.update()
        swerveModules.update()
        logModuleStates("Swerve State", swerveModules.states.asArray())
        logModuleStates("Swerve Set State", arrayOf(
                swerveModules.frontLeft.setState,
                swerveModules.frontRight.setState,
                swerveModules.rearLeft.setState,
                swerveModules.rearRight.setState
        ))
        SmartDashboard.putNumberArray("Swerve Module Distance", doubleArrayOf(
                swerveModules.frontLeft.position.distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.frontRight.position.distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.rearLeft.position.distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.rearRight.position.distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS))
        SmartDashboard.putNumberArray("Swerve Module Distance Revolutions", doubleArrayOf(
                swerveModules.frontLeft.position.distanceMeters,
                swerveModules.frontRight.position.distanceMeters,
                swerveModules.rearLeft.position.distanceMeters,
                swerveModules.rearRight.position.distanceMeters))
    }

    private fun logModuleStates(key: String, swerveModuleStates: Array<SwerveModuleState>) {
        val dataList: MutableList<Double> = ArrayList()
        for (swerveModuleState in swerveModuleStates) {
            dataList.add(swerveModuleState.angle.radians)
            dataList.add(swerveModuleState.speedMetersPerSecond)
        }
        SmartDashboard.putNumberArray(key,
                dataList.stream().mapToDouble { obj: Double -> obj }.toArray())
    }

    val rotation: Rotation2d?
        get() {
            var rot: Rotation2d? = gyro.angle
            if (DriveConstants.GYRO_REVERSED) {
                rot = rot!!.unaryMinus()
            }
            return rot
        }
    val rotation3d: Rotation3d
        get() = gyro.rotation3d
    val modulePositions: Array<SwerveModulePosition>
        get() = swerveModules.positions.asArray()

    /**
     * Method to drive the drivetrain using chassis speeds.
     *
     * @param speeds The chassis speeds.
     */
    fun drive(speeds: ChassisSpeeds?) {
        val swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
        setModuleStates(swerveModuleStates)
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    fun setX() {
        swerveModules.setDesiredStates(
                States(
                        SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),  // front right
                        SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),  // rear left
                        SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),  // rear right
                        SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
                ).asArray()
        )
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND)
        swerveModules.setDesiredStates(desiredStates)
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    fun resetEncoders() {
        swerveModules.resetEncoders()
    }

    /**
     * Zeroes the heading of the robot.
     */
    fun zeroHeading() {
        gyro.reset()
    }

    val turnRate: Double
        get() = gyro.rate.degrees
}