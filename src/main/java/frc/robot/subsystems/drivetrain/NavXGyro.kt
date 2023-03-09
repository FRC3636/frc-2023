package frc.robot.subsystems.drivetrain

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import frc.robot.Constants.DriveConstants

class NavXGyro : Gyro {
    private val navX = AHRS()
    override val connected: Boolean
        get() = navX.isConnected
    override val angle: Rotation2d
        get() = navX.rotation2d
    override val rotation3d: Rotation3d
        get() = DriveConstants.GYRO_ROTATION.rotateBy(
                Rotation3d(
                        Quaternion(
                                navX.quaternionW.toDouble(),
                                navX.quaternionX.toDouble(),
                                navX.quaternionY.toDouble(),
                                navX.quaternionZ.toDouble()
                        )
                )
        ).rotateBy(DriveConstants.GYRO_ROTATION.unaryMinus())
    override val rate: Rotation2d
        get() = Rotation2d.fromDegrees(navX.rate)

    init {
        navX.calibrate()
        reset()
    }

    override fun setGyroRotation(rotation: Rotation2d) {
        navX.reset()
        navX.angleAdjustment = -rotation.degrees // Probably non-functional
    }

    override fun update() {}
    override fun reset() {
        navX.reset()
    }
}