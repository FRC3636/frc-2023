package frc.robot.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.Constants.FieldConstants

object AllianceUtils {
    @JvmStatic
    fun allianceToField(alliancePose: Pose2d): Pose2d? {
        return when (DriverStation.getAlliance()) {
            Alliance.Blue -> alliancePose
            Alliance.Red -> Pose2d(
                    Translation2d(
                            FieldConstants.fieldLength - alliancePose.x,
                            alliancePose.y
                    ),
                    alliancePose.rotation.unaryMinus()
            )

            else -> null
        }
    }

    val isBlue: Boolean
        get() = DriverStation.getAlliance() == Alliance.Blue
    @JvmStatic
    val fieldOrientationZero: Rotation2d
        get() = Rotation2d.fromRadians(if (isBlue) 0.0 else Math.PI)
}