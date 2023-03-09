package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState

interface SwerveModule {
    val position: SwerveModulePosition
    fun setDesiredState(desiredState: SwerveModuleState)
    val state: SwerveModuleState
    val setState: SwerveModuleState
    val swerveEncoderPosition: Double
    fun resetEncoders()
    fun update()
}