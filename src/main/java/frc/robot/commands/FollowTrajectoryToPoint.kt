package frc.robot.commands

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.Constants.AutoConstants
import frc.robot.RobotContainer
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain

//Uses PathPlanner to move the robot to the specified Pose2d
class FollowTrajectoryToPoint(private val drivetrain: Drivetrain, private val poseEstimation: PoseEstimation, private val target: Pose2d?) : Command {
    private var swerveControllerCommand: PPSwerveControllerCommand? = null
    override fun initialize() {
        val trajectory: PathPlannerTrajectory = buildTrajectory(target)
        RobotContainer.field.getObject("Alignment Target").pose = trajectory.endState.poseMeters
        RobotContainer.field.robotObject.setTrajectory(trajectory)
        RobotContainer.field.getObject("Target").pose = target
        swerveControllerCommand = PPSwerveControllerCommand(
                trajectory,
                poseEstimation::estimatedPose,
                PIDController(AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0),
                PIDController(AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0),
                PIDController(AutoConstants.P_THETA_PATH_CONTROLLER, 0.0, 0.0), { speeds: ChassisSpeeds? -> drivetrain.drive(speeds) })
        swerveControllerCommand!!.initialize()
    }

    private fun buildTrajectory(target: Pose2d?): PathPlannerTrajectory {
        val initial = poseEstimation.estimatedPose
        val initialV = poseEstimation.estimatedVelocity
        return PathPlanner.generatePath(
                PathConstraints(
                        AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                        AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                ),
                PathPoint(
                        initial.translation,
                        if (initialV.norm == 0.0) target!!.translation.minus(initial.translation).angle else initialV.angle,
                        initial.rotation,
                        initialV.norm),
                PathPoint(
                        target!!.translation,
                        target.translation.minus(initial.translation).angle,
                        target.rotation)
        )
    }

    override fun execute() {
        swerveControllerCommand!!.execute()
    }

    override fun end(interrupted: Boolean) {
        swerveControllerCommand!!.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return swerveControllerCommand!!.isFinished
    }

    override fun getRequirements(): Set<Subsystem> {
        return mutableSetOf<Subsystem>(drivetrain)
    }
}