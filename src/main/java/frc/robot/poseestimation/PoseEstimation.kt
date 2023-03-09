package frc.robot.poseestimation

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import frc.robot.Constants
import frc.robot.Constants.DriveConstants
import frc.robot.RobotContainer
import frc.robot.poseestimation.VisionBackend.Measurement

class PoseEstimation {
    private var photonVision: PhotonVisionBackend? = null
    private val limelight: LimelightBackend
    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            RobotContainer.drivetrain.rotation,
            RobotContainer.drivetrain.modulePositions,
            Pose2d(),
            DriveConstants.ODOMETRY_STD_DEV,
            VecBuilder.fill(0.0, 0.0, 0.0) // will be overwritten for each measurement
    )
    private val usePhotonVisionEntry = RobotContainer.autoTab.add("Use PhotonVision", true).withWidget(BuiltInWidgets.kToggleButton).entry
    private val useLimelightEntry = RobotContainer.autoTab.add("Use Limelight", false).withWidget(BuiltInWidgets.kToggleButton).entry
    private val poseHistory = TimeInterpolatableBuffer.createBuffer<Pose2d>(1.5)

    init {
        try {
            photonVision = PhotonVisionBackend()
        } catch (e: Exception) {
            println("Failed to initialize PhotonVision")
            e.printStackTrace()
        }
        limelight = LimelightBackend()
    }

    fun periodic() {
        poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.estimatedPosition)
        if (usePhotonVisionEntry.getBoolean(false)) {
            try {
                photonVision!!.measurement.filter { measurement: Measurement? -> measurement!!.ambiguity < Constants.VisionConstants.AMBIGUITY_FILTER }.ifPresent { measurement: Measurement? -> addVisionMeasurement(measurement) }
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
        if (useLimelightEntry.getBoolean(false)) {
            limelight.measurement.ifPresent { measurement: Measurement? -> addVisionMeasurement(measurement) }
        }
        RobotContainer.field.robotPose = estimatedPose
    }

    fun updateOdometry(gyro: Rotation2d?, modulePositions: Array<SwerveModulePosition>) {
        poseEstimator.update(gyro, modulePositions)
    }

    val estimatedPose: Pose2d
        get() = poseEstimator.estimatedPosition
    val estimatedVelocity: Translation2d
        get() {
            val now = Timer.getFPGATimestamp()
            val current = poseHistory.getSample(now).get().translation
            val previous = poseHistory.getSample(now - DIFFERENTIATION_TIME).get().translation
            return current.minus(previous).div(DIFFERENTIATION_TIME)
        }

    fun resetPose(pose: Pose2d?) {
        poseEstimator.resetPosition(RobotContainer.drivetrain.rotation, RobotContainer.drivetrain.modulePositions, pose)
    }

    private fun addVisionMeasurement(measurement: Measurement?) {
        poseEstimator.addVisionMeasurement(measurement!!.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation)
    }

    companion object {
        private const val DIFFERENTIATION_TIME: Double = TimedRobot.kDefaultPeriod
    }
}