package frc.robot.commands.alignment

import frc.robot.commands.NavigateToPoint
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.Node

//Navigate to the point in front of the node specified
class AlignToNode(drivetrain: Drivetrain, poseEstimation: PoseEstimation, targetNode: Node) : NavigateToPoint(drivetrain, poseEstimation, targetNode.nodePose!!.transformBy(targetNode.robotOffset))