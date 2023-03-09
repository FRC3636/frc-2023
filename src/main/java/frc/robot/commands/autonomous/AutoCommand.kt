package frc.robot.commands.autonomous

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants.AutoConstants
import frc.robot.Constants.DriveConstants
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain

object AutoCommand {
    var eventMap = mapOf(
            "print" to InstantCommand({ println("print event triggered") })
    )

    fun makeAutoCommand(drivetrain: Drivetrain, poseEstimation: PoseEstimation, name: String?): Command {
        val pathGroup = PathPlanner.loadPathGroup(name, PathConstraints(AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED))

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        val autoBuilder = SwerveAutoBuilder(
                poseEstimation::estimatedPose, { pose: Pose2d? -> poseEstimation.resetPose(pose) },  // Pose2d consumer, used to reset odometry at the beginning of auto
                DriveConstants.DRIVE_KINEMATICS,  // SwerveDriveKinematics
                PIDConstants(AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0),  // PID constants to correct for translation error (used to create the X and Y PID controllers)
                PIDConstants(AutoConstants.P_THETA_PATH_CONTROLLER, 0.0, 0.0), { desiredStates: Array<SwerveModuleState> -> drivetrain.setModuleStates(desiredStates) },  // Module states consumer used to output to the drive subsystem
                eventMap,
                true,  // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
        )
        return autoBuilder.fullAuto(pathGroup)
    }
}