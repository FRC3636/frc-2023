package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d

interface Gyro {
    val connected: Boolean
    fun setGyroRotation(rotation: Rotation2d)
    fun update()
    fun reset()
    val angle: Rotation2d
    val rate: Rotation2d
    val rotation3d: Rotation3d
}