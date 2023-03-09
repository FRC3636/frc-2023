package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain

//Moves the robot to a specified Pose2d
//Uses FollowTrajectoryToPoint for most of the movement,
//and PIDDriveToPoint at the end to correct for any error
open class NavigateToPoint(drivetrain: Drivetrain, poseEstimation: PoseEstimation, target: Pose2d?) : SequentialCommandGroup(
        FollowTrajectoryToPoint(drivetrain, poseEstimation, target),
        PIDDriveToPoint(drivetrain, poseEstimation, target)
)