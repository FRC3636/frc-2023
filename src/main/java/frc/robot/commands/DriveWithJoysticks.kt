package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.Constants.DriveConstants
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.AllianceUtils

class DriveWithJoysticks(private val drivetrain: Drivetrain, private val poseEstimation: PoseEstimation, private val translation: Joystick, private val rotation: Joystick) : Command {
    private var fieldOrientationZero = AllianceUtils.fieldOrientationZero

    init {
        drivetrain.resetEncoders()
    }

    override fun execute() {
        // Negative because joysticks are inverted
        val tx = MathUtil.applyDeadband(-translation.y * (translation.z + 1) / 2, 0.15)
        val ty = MathUtil.applyDeadband(-translation.x * (translation.z + 1) / 2, 0.15)
        val r = MathUtil.applyDeadband(-rotation.x * (rotation.z + 1) / 2, 0.15)
        val vx = tx * DriveConstants.MAX_SPEED_METERS_PER_SECOND
        val vy = ty * DriveConstants.MAX_SPEED_METERS_PER_SECOND
        val omega = r * DriveConstants.MAX_ANGULAR_SPEED
        val fieldRelSpeeds = ChassisSpeeds(vx, vy, omega)
        val robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelSpeeds, poseEstimation.estimatedPose.rotation.minus(fieldOrientationZero))
        drivetrain.drive(robotRelSpeeds)
    }

    override fun getRequirements(): Set<Subsystem> {
        return mutableSetOf<Subsystem>(drivetrain)
    }

    fun resetFieldOrientation() {
        fieldOrientationZero = poseEstimation.estimatedPose.rotation
    }
}