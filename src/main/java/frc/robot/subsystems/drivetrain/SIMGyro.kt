package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.TimedRobot
import frc.robot.Constants.DriveConstants
import java.util.*

class SIMGyro(private val swerveModules: SwerveModules) : Gyro {
    var rotation = Rotation2d()

    override var rate = Rotation2d()
    override val angle: Rotation2d
        get() = rotation
    override val rotation3d: Rotation3d
        get() = Rotation3d()
    override val connected: Boolean
        get() = true

    override fun setGyroRotation(rotation: Rotation2d) {
        this.rotation = rotation
    }

    override fun update() {
        val moduleStates = swerveModules.states.asArray()
        val velocity = Arrays.stream(moduleStates).map { state: SwerveModuleState? -> Translation2d(state!!.speedMetersPerSecond, state.angle) }.reduce { obj: Translation2d, other: Translation2d? -> obj.plus(other) }.orElseThrow().div(moduleStates.size.toDouble())
        val referenceModule = swerveModules.states.frontLeft
        val referenceModulePosition = Translation2d(DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE).div(2.0)
        val referenceModuleVelocity = Translation2d(referenceModule!!.speedMetersPerSecond, referenceModule.angle)
        val referenceRotationalVelocityComponent = referenceModuleVelocity.minus(velocity)

        // FIXME: this is a really scuffed way of doing this
        val turningDirection = (if (referenceRotationalVelocityComponent.angle.radians > referenceModulePosition.angle.radians) 1 else -1).toDouble()
        rate = Rotation2d.fromRadians(referenceRotationalVelocityComponent.norm * turningDirection / referenceModulePosition.norm)
        rotation = rotation.plus(rate.times(TimedRobot.kDefaultPeriod))
    }

    override fun reset() {
        rotation = Rotation2d()
    }
}